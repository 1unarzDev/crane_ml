# crane-archive: the cluster's cold storage

The coordinator keeps new artifacts in R2 (hot). An archive node keeps everything, deduplicated by sha256, for
as long as the disks hold it (cold). The design and its measurements are in `design-storage.md` (research cache),
and the coordinator side is `coordinator/src/archive.ts`.

```
coordinator (Worker, R2, D1)                         archive node (any Linux host; innovation today)
  GET  /v1/archive/pending   <-------- pull ------   crane-archive run (systemd --user)
  GET  /v1/artifact/<key>    <-------- download --     store: ~/crane-archive (plain dir) -> /tank/crane-archive (ZFS)
  POST /v1/archive/confirm   <-------- after fsync     scrub, quota, heartbeat
  sweep: R2 delete once archived, past HOT_DAYS[kind], not protected
  reads: R2, else ---- fetch https://crane-archive.lunarz.dev/o/<sha256> (Access service token) ---> tunnel -> 127.0.0.1:7420
```

## Tiers

| kind | in R2 for | archive codec | evictable from the archive |
|---|---|---|---|
| `ctraj` (run.ctraj) | 60 days | as is | never |
| `lod` (run.lod.ctraj) | 365 days | as is (packed) | never |
| `summary` (run.json, result.json, ...) | 365 days | zstd-19 when it pays | never |
| `raw` (logs, jsonl) | 14 days (`raw_days`) | zstd-19 when it pays | after 365 days, above 80% full |
| `bundle` (*.tar.zst) | 14 days | as is | after 365 days |
| `crane` (full state) | 7 days | zstd-19 (5.2x) | after 180 days |
| `render_master` (chunks, h264) | 14 days | as is | after 30 days (re-renderable) |
| `render_final` (h265, gif) | 90 days | as is | never |

Change a window with `PUT /v1/admin/retention {"hot_days": {"ctraj": 30}}`. Pinned artifacts, every artifact
of each study's best run and each iteration's best trajectories stay in R2 whatever their age (and are archived
too: two copies, two sites). Without an archive node the old rule holds: raw items are deleted after
`raw_days`, summaries are kept. If the archive stops heartbeating for 72 h, or lags by more than
`ARCHIVE_MAX_LAG_GB`, items are held in R2 until R2 passes `R2_SOFT_CAP_GB` (200 GB); only then are expired
raw items deleted, and the count shows in `GET /v1/admin/archive`. Public state: `GET /v1/feed/archive`.

A read of an artifact that left R2 (`GET /v1/artifact/<key>`, Range supported) is fetched from the archive
through the Worker. Trajectories and summaries come back into R2 for another hot window. With the node offline
the answer is `503`, `Retry-After: 300`, `{"tier": "archive", "state": "offline"}`.

## Store

```
<root>/.crane-archive-root   marker; the daemon refuses a root without it (an unmounted pool stays empty)
<root>/objects/ab/cd/<sha>   objects >= 64 KiB stored, one file each (.zst / .xz when compressed)
<root>/packs/YYYY/MM/DD-NNNN.pack   smaller objects appended (sealed at 256 MiB or daily); random access by offset
<root>/index.sqlite          sha256 -> path, offset, length, codec, size, kind, first_seen, verified_at
```

Dedup is exact (same bytes, one copy: course truths, repeated logs, re-uploads). Writes are staged, hashed,
compressed, fsynced and renamed before the coordinator hears of them. The scrub rehashes `scrub.gb_per_day`
a day, oldest-verified first. A mismatch is quarantined and reported; the coordinator re-queues it while R2 still
has a copy, otherwise marks it gone. The quota is `min(quota_bytes, disk free - min_free_bytes)`. Above 80% the
daemon evicts the evictable kinds above, oldest first, to 75%; above 90% it pauses pulls of evictable kinds.

## Install (innovation)

```sh
cluster/archive/install.sh --root ~/crane-archive --node innovation --quota 20G --min-free 25G
# admin: an archive token (read artifacts and /v1/archive/* only)
curl -s -X POST -H "authorization: Bearer $ADMIN_TOKEN" -d '{"name":"innovation"}' \
  https://crane.lunarz.dev/v1/admin/archive_token | python3 -c 'import json,sys; print(json.load(sys.stdin)["token"])' \
  > ~/.config/crane-cluster/archive.token && chmod 600 ~/.config/crane-cluster/archive.token
cluster/archive/install.sh            # again: starts crane-archive.service
crane-archive status                  # ~/.local/share/crane-cluster/bin/crane-archive
```

innovation's system disk is 97% used, so the plain-directory store starts with a 20 GB quota and keeps
25 GB free; at today's volume (about 0.4 GB/day, mostly renders) that is weeks of history before eviction.

## Serving

`CLOUDFLARE_API_TOKEN=... python3 cluster/archive/cloudflare_setup.py` creates (or finds) the tunnel
`crane-archive`, its ingress `crane-archive.lunarz.dev -> 127.0.0.1:7420`, the DNS record, the service token
`crane-coordinator-archive` and an Access application that admits only that token. It writes the connector
token, the service token's id and secret (`~/.config/crane-cluster/secrets.env`) and the Access team and AUD
into `archive.yaml`, so the daemon also checks each request's `Cf-Access-Jwt-Assertion`. Then:

```sh
cluster/archive/install.sh            # starts crane-archive-tunnel.service (cloudflared, token file)
cd cluster/coordinator
. ~/.config/crane-cluster/secrets.env
printf %s "$ARCHIVE_ACCESS_ID" | npx wrangler secret put ARCHIVE_ACCESS_ID
printf %s "$ARCHIVE_ACCESS_SECRET" | npx wrangler secret put ARCHIVE_ACCESS_SECRET
```

`ARCHIVE_URL` is a var in `wrangler.toml`. Moving the node to another host: install there, copy the store
(`crane-archive move` or rsync), copy the tunnel token; no DNS or Worker change.

## When the drives arrive

1. Build the pool (below), `zfs create -o compression=lz4 -o recordsize=1M -o atime=off -o xattr=sa tank/crane-archive`,
   and `chown` it to the user.
2. `crane-archive move --to /tank/crane-archive`: copies objects, packs and the index, verifies every sha256 at
   the destination, and switches `root` in `archive.yaml` (the old root is kept until you delete it).
3. Set `quota_bytes` to the pool's usable size (or a bit under), re-run `install.sh` (the unit's
   `RequiresMountsFor` follows the new root), and add `smart: [/dev/disk/by-id/...]` for SMART in the heartbeat.
4. smartd (short daily, long weekly), a monthly `zpool scrub`, and a UPS with NUT that stops the daemon and
   exports the pool on low battery.

### Hardware (design-storage.md 3.1)

SAS drives do not work in USB enclosures: USB bridges speak SATA only. Pick one:

- **A, SAS (best $/TB, if innovation has a free PCIe x8 slot):** an LSI SAS2308/SAS3008 HBA in IT mode
  (9207-8e or 9300-8e), an external 4-bay SAS JBOD, 4 x 8 TB used enterprise SAS, ZFS raidz2 (about 16 TB
  usable; any two drives may fail). About $600-900 with a UPS.
- **B, USB (plug into any host, laptops included):** a 4-bay USB 3.2 Gen 2 (10 Gb/s, UASP) enclosure with
  per-bay pass-through and SMART pass-through, 2 x 16 TB enterprise **SATA** as a ZFS mirror (16 TB usable,
  grow with a second mirror). About $600-850 with a UPS.
- **C, minimum:** one 18 TB external USB drive, no redundancy (the protected set's second copy is in R2).

At the mid scenario (10k fast-sim runs, 500 Unity runs, 4 renders a day) the archive grows about 0.6 TB a year;
16 TB usable lasts over a decade. RAID is not a backup: losing the array loses unprotected history (old non-best
runs, raw logs, non-hero renders), which can be rerun by seed and params but not bit for bit.

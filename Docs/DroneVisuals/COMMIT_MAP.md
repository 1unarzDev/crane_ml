# Commit map: public filtered copy of research/drone-sim (2026-11-10)

On 2026-11-10 the remote research/drone-sim was replaced by a filtered copy (`~/.cache/research/fs-scripts/publish-drone-sim`). The filter does three things:
- removes Claude attribution trailers from commit messages
- strips venue and paper-identifying text for double-blind review
- drops `mission/VENUES.md`

The local branch keeps the original history (backup ref `refs/archive/drone-sim-pre-rewrite`). Any doc or manifest that cites an old sha refers to the left column.

| old (local) | new (public) |
|---|---|
| 0a26c1dc787d | db5c1924c5d7 |
| 12417862739f | 98a3a7a63cae |
| 1e04b8906eed | 7206470c333e |
| 2b8c02f513e4 | 9d6a80d47862 |
| 2f9fe996ca4e | 8a25eff7a282 |
| 34ae08b588dc | 54378743ba29 |
| 404294777f83 | dc9c154770b2 |
| 463185167db7 | 08e00b3324dd |
| 50d9616b1845 | 0800a81c59c6 |
| 56dceb05986f | 7202d32a4bfe |
| 59daf6e035bf | a070686ca89e |
| 6055a11a4b9c | 8f16bfa575c9 |
| 6728637418cf | d38a6ccd448f |
| 6bc182f2dd60 | 1e109b556220 |
| 7c6e4da96161 | 71d7b22609b6 |
| 8f9aca9ef610 | d4fbcd0ae533 |
| 9d95669de448 | efed8dfb2679 |
| a6d96b3a539c | 6d47aefb023a |
| b5f98a0cbbe6 | 4c89414e516d |
| bb6e314dbc52 | c6a35497c756 |
| c1bacf9eb5dd | 19bad32be6c7 |
| d5ef006ddda9 | 1c906c3ff17b |
| e9123c6cf498 | 03d931cc5ade |

`flights/seed1337_run1/manifest.json` keeps `crane_git_sha` = 67286374 (the local original). Its public equivalent is d38a6ccd448f.

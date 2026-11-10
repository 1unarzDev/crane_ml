# Changelog (Crane.Fleet.Docking)

## 2026-11-10
- Accept `fleet_profile_v1.json`: provenance read from `provenance_levels` (`concept`, `datasheet` added to `Provenance`); `hook_count` 1 selects the reciprocal one-hook-plus-one-pin face mode (HOOK_A = initiator's own hook at -y, HOOK_B = partner's at +y, partner report and close command delayed by `PartnerLinkLatency`, stale report fails safe and blocks release of a verified connection); v1 magnet geometry, hook open angle, per-role active faces and descriptive fields (metadata). v0 loads to identical values (golden-tested) and its behaviour is unchanged.

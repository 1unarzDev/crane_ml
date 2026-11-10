# Changelog (Crane.Fleet.Perception)

## 2026-11-10
- Accept `fleet_profile_v1.json` (v0 parses to identical values, golden-tested). Descriptive leaves (motor, magnet, compute, mcu, lifter option, servo, partner acquisition) are carried as `FleetProfile.Metadata`, never coerced; non-numeric values on numeric leaves fail with the field path. New sensors: RealSense D435 (depth `DepthModel`, infra1/infra2 as two mono imagers at 50 mm), RPLIDAR C1 via the existing 2-D lidar, Arducam OV9281 mast camera; CAD v1 mount poses; `RoverSpec` treats the D435 and mast camera as a front camera.

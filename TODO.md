# Position Mapping Optimization TODO

## Phase 1: Update ReadAprilTag.java
- [x] Add methods to return vision metadata (tag count, average distance, ambiguity)
- [x] Return all tracked targets for multi-tag analysis

## Phase 2: Update VisionSubsystem.java
- [x] Implement dynamic stdDev calculation based on:
  - Number of tags visible
  - Pose ambiguity
  - Distance to tags
  - Robot velocity
- [x] Add pose outlier rejection (reject if > 2m from odometry)
- [x] Enable vision rotation when multiple tags visible
- [x] Fix bump detection trust logic (inverted!)
- [x] Add timestamp validation for stale measurements

## Phase 3: Testing & Tuning
- [ ] Test position accuracy
- [ ] Tune stdDev multipliers
- [ ] Verify rotation updates work correctly

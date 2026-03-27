# Shooter & Turret Improvements TODO

## Current Status
- [x] Analyzed subsystems (TurretSubsystem, ShooterSubsystem, Hood unused)
- [x] Reviewed Constants.java (RPM spline confirmed ★ points)
- [x] Reviewed RobotContainer.java (bindings, vision→RPM pipeline)

## Improvement Plan

### 1. SysId Setup for Flywheel Tuning [Priority 1 - In Progress]
- [ ] Add SysId buttons to RobotContainer (op D-pad or buttons)
- [ ] Run forward/reverse quasistatic/dynamic on field
- [ ] Update Constants.FLYWHEEL_kS/kV/kA from analyzer
- Dependent: ShooterSubsystem (already supports)

### 2. Flywheel Stall Detection [Priority 2]
- [ ] Add periodic() check in ShooterSubsystem: if RPM < 50% target → fault + stop
- [ ] Log to AdvantageScope/Shuffleboard

### 3. RPM Table Completion [Priority 3 - Field Test]
- [ ] Confirm 5.15m VFAR RPM (expected ~3600?)
- [ ] Add ★ entry to RPM_DISTANCE_TABLE + spline anchors
- [ ] Test/extrapolate 0.8m close-shot if needed

### 4. Additional Enhancements
- [ ] Safe Stow command (turret 0° + idle flywheel)
- [ ] SOTM teleop toggle (DriveToHubAndShoot)
- [ ] Turret fault status (hardware faults → disable aim)

## Next Step
Add SysId buttons to RobotContainer.java

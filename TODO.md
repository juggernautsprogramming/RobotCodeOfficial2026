# Task: Add Intake Roller Motors (CAN 27/28)

## Plan Summary
- Add left/right intake rollers to IntakeAdapter for coordinated control with existing uptake/actuation.
- CAN Bus: ChassisCAN (standard).
- Control: DutyCycleOut power ±0.8, matching existing ROLLER_POWER.
- Config: Copy from UptakeSubsystem (brake, PID slot0, voltage ±8V).

## Steps
- [ ] Step 1: Add `IntakeConstants` class to `src/main/java/frc/robot/Constants.java`
- [ ] Step 2: Update `src/main/java/frc/robot/subsystems/Intake/IntakeAdapter.java`
  - Add imports
  - Add motor fields + DutyCycleOut requests
  - Init/configure motors in constructor
  - Update `run()`, `eject()`, `stop()` to control new motors
- [ ] Step 3: Test integration
  - Deploy robot
  - Test A button: all rollers spin forward + arm deploys
  - Test LB: all rollers reverse
  - Tune inversion/power if needed via constants

## Notes
- Existing: Uptake=42 (CCW+), Actuation=26 (CW+)
- New: Left=27 (CCW+), Right=28 (CW+) - verify wiring

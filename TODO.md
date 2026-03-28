# Climber Power Issue Fix - Elevator Not Reaching Max Rotation

## Current Status
- [x] Diagnosis: Open-loop duty cycle, no voltage comp, 45A limit too low
- [x] Update Constants.java (STATOR_LIMIT_AMPS=80, UP_POSITION=2.0 rot)
- [x] Update ClimberSubsystem.java (add voltage comp 12V, update limit)
- [ ] Test: Run ClimberManualTuneCommand or ClimberGoUpCommand
- [ ] Monitor: Shuffleboard pos/vel/current, battery voltage
- [ ] Mechanical: Check friction/binding at top if still insufficient
- [ ] Close issue

## Testing Commands
- Manual: ClimberManualTuneCommand (joystick power)
- Auto: ClimberGoUpCommand (check power setpoint)

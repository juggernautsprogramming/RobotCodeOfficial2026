# Quick Reference: Building Move-and-Shoot Autos

## TL;DR - Quick Steps

1. **Open PathPlanner Desktop App**
2. **Click "Auto" tab** at the bottom
3. **Create New Auto** (give it a name)
4. **Add commands in order:**
   ```
   [Path] → Drive somewhere
   [Named Command] → Do action (Shoot_Mid, StartIntake, etc.)
   [Path] → Drive somewhere else (optional)
   [Named Command] → Another action
   ```
5. **Save** (auto-saves to `src/main/deploy/pathplanner/autos/`)
6. **Deploy** with `./gradlew deploy`
7. **Select auto in Elastic** → "Auto Chooser" dropdown
8. **Enable on Driver Station** → Watch it go!

---

## Command Cheat Sheet

### Shooting (Self-Contained)
```
Shoot_Close    → 1.5 m, 2440 RPM  (spins up → waits → feeds → stops)
Shoot_Mid      → 2.5 m, 2610 RPM
Shoot_Far      → 4.0 m, 3000 RPM
Shoot_VFar     → 5.5 m, 3600 RPM
IdleFlywheel   → Stops flywheel (use at end of auto)
```

### Intake
```
StartIntake    → Deploys arm + runs all rollers
StopIntake     → Retracts arm + stops rollers
Eject          → Reverses intake
```

### Turret (Advanced)
```
TurretAutoAim        → Uses odometry + vision (good all-around)
TurretAprilTagAim    → Uses only camera (when tag visible)
```

### Other
```
AutoClimbLevel1      → Auto climb (3s timeout, use at end)
StartFeeder / StopFeeder (manual control, usually not needed)
```

---

## Simple Patterns

### "Just Shoot"
```
Shoot_Mid
IdleFlywheel
```

### "Drive Back & Shoot"
```
Path: Position1ToHub2.5meters
Shoot_Mid
IdleFlywheel
```

### "Move, Pick Up, Shoot"
```
Path: SomewhereWithBall
StartIntake
Wait: 1.5 seconds
StopIntake
Path: HubLocation
Shoot_Mid
IdleFlywheel
```

### "Multi-Shot"
```
Path: Position1ToHub2.5meters
Shoot_Mid
Path: Position2ToHub4meters
Shoot_Far
IdleFlywheel
```

### "Shoot & Climb"
```
Shoot_Close
IdleFlywheel
Path: ToClimber
AutoClimbLevel1
```

---

## Your Existing Auto Files

You already have these working:
- **Pos1ToShoot.auto** → Drive 2.5m back, shoot mid, idle
- **Pose2ShootToClimb.auto** → Drive 1.5m, shoot close, drive to climber

---

## Test Autos (Added)

- **TestSimpleShoot.auto** → Just shoots (no movement)
- **TestMoveAndShoot.auto** → Moves back 2.5m then shoots
- **TestIntakeAndShoot.auto** → Runs intake for 2s then shoots

Use these to verify everything works!

---

## Key Things to Remember

✅ **Sequential means in order** — commands execute one after another
✅ **Paths must exist** — Create them in PathPlanner first, use exact name
✅ **Named commands are case-sensitive** — `Shoot_Mid` not `Shoot_mid`
✅ **Always end with `IdleFlywheel`** — So shooter doesn't keep spinning
✅ **resetOdom: true** — Zeros odometry at auto start (keep it true)
✅ **Test first, compete second** — Verify each part works individually

---

## Next: Create Your Strategy

1. **What balls can you reach?** → Design paths to them
2. **How many can you shoot?** → Plan pickup/shoot cycles
3. **Where's the climber?** → Add climb at end if time allows
4. **Build the auto sequence** → Combine paths + commands

---

## Deployment

After creating/editing autos:

```bash
cd C:\Users\Admin\Desktop\Robot2026Code
./gradlew deploy
```

Then in Elastic/Shuffleboard:
- **Dashboard** → "Auto Chooser"
- **Select your auto** from dropdown
- **Enable robot** in auto mode
- **Watch!** 🚀

---

## Troubleshooting

| Issue | Fix |
|-------|-----|
| Auto doesn't appear | Save file as `.auto` in correct folder, redeploy |
| Robot doesn't move | Check path name matches exactly, path file exists |
| Shooter doesn't spin | Make sure `Shoot_*` command name is exact, check RPM settings |
| Turret doesn't aim | Zero turret in teleop first, check camera is connected |
| Auto stops early | Check for timeouts (some commands have 5s limit), add Wait if needed |

Enjoy! 🤖

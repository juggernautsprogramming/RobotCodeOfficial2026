# PathPlanner Auto Building: Step-by-Step Visual Guide

## What You'll Learn

By the end, you'll be able to build autonomous routines that:
- Move the robot to specific locations
- Pick up balls
- Shoot at different distances
- Climb the rung

---

## Part 1: Understanding the Basics

### The Three Building Blocks

Your autos are made from three types of commands:

#### 1. **Path** 🛣️
- Robot drives from Point A to Point B
- You create these in PathPlanner's path editor
- Examples: `Position1ToHub2.5meters`, `HubToClimber`

#### 2. **Named Command** ⚙️
- A pre-programmed action
- Run by the robot during autonomous
- Examples: `Shoot_Mid`, `StartIntake`, `IdleFlywheel`

#### 3. **Wait** ⏱️
- Just pauses for X seconds
- Useful for waiting for ball to be picked up
- Example: Wait 1.5 seconds for intake

### How They Execute

Sequential execution means they happen **in order, one after another**:

```
Step 1: Execute Path "Position1ToHub2.5meters"
         └─ Robot drives backward 2.5 meters
        
Step 2: Execute Named Command "Shoot_Mid"
         └─ Flywheel spins to 2610 RPM
         └─ Wait until ready (≤2 sec)
         └─ Feeder runs for ~0.5 sec
         └─ Feeder stops
         └─ Complete!

Step 3: Execute Named Command "IdleFlywheel"
         └─ Flywheel stops
         └─ Done!
```

---

## Part 2: Your Robot's Capabilities

### What Can You Do?

#### **Movement** 🤖
- Drive in any direction
- Rotate to any angle
- Follow complex curved paths

#### **Shooting** 🎯
- Auto spin-up to target RPM
- Wait for flywheel to reach RPM
- Auto-feed the ball
- Stop cleanly

#### **Intake** 🍔
- Deploy arm
- Run rollers to pick up balls
- Retract arm and stop

#### **Climbing** 🧗
- Auto-climb to Level 1 rung

#### **Advanced** 🔬
- Aim turret while moving
- Blend odometry + vision
- Coordinate multiple systems

---

## Part 3: Creating Your First Auto

### Step-by-Step: "Drive Back and Shoot"

#### Step 1: Open PathPlanner Desktop

```
File → Open
Navigate to: C:\Users\Admin\Desktop\Robot2026Code\src\main\deploy\pathplanner
```

#### Step 2: Switch to "Auto" Tab

- Look at the bottom of PathPlanner
- See tabs: "Path", "Auto", "Autos"
- Click **"Auto"** tab

#### Step 3: Create New Auto

- Click the **"+"** button in the Auto section
- Enter name: `MyFirstAuto`
- Press Enter

#### Step 4: Add First Command - Drive Path

- In the Auto editor, click **"Add Command"** or **"+"**
- Select **"Path"**
- From dropdown, choose **"Position1ToHub2.5meters"**
- This makes robot drive backward 2.5 meters

#### Step 5: Add Second Command - Shoot

- Click **"Add Command"** again
- Select **"Named Command"**
- From dropdown, choose **"Shoot_Mid"**
- This makes robot spin up, wait, then feed

#### Step 6: Add Third Command - Stop Flywheel

- Click **"Add Command"** again
- Select **"Named Command"**
- From dropdown, choose **"IdleFlywheel"**
- This stops the flywheel

#### Step 7: Set Options

At the top of the auto editor:
- **resetOdom**: Keep it **TRUE** (zeros starting position)
- **choreoAuto**: Keep it **FALSE** (unless using Choreo)

#### Step 8: Save

- Press **Ctrl+S** (or File → Save)
- File is saved to `src/main/deploy/pathplanner/autos/MyFirstAuto.auto`

---

## Part 4: Understanding the JSON Format

If you edit autos in VS Code, here's what's happening:

```json
{
  "version": "2025.0",              // PathPlanner format version
  "command": {
    "type": "sequential",            // Commands execute in order
    "data": {
      "commands": [
        {
          "type": "path",            // This is a path command
          "data": {
            "pathName": "Position1ToHub2.5meters"
          }
        },
        {
          "type": "named",           // This is a named command
          "data": {
            "name": "Shoot_Mid"      // Exact name, case-sensitive!
          }
        },
        {
          "type": "named",
          "data": {
            "name": "IdleFlywheel"
          }
        }
      ]
    }
  },
  "resetOdom": true,                 // Zero odometry at start
  "folder": null,
  "choreoAuto": false
}
```

---

## Part 5: Common Patterns

### Pattern 1: Just Shoot (No Movement)

```
Named Command: Shoot_Mid
Named Command: IdleFlywheel
```

**When to use:** You're already at the right distance, just need to shoot

---

### Pattern 2: Drive Then Shoot

```
Path: [Your path to hub]
Named Command: Shoot_Mid
Named Command: IdleFlywheel
```

**When to use:** Start in safe location, drive to optimal shooting distance

---

### Pattern 3: Pickup and Shoot

```
Named Command: StartIntake
Wait: 1.5 seconds (time to pick up ball)
Named Command: StopIntake
Named Command: Shoot_Mid
Named Command: IdleFlywheel
```

**When to use:** Robot starts with a ball nearby, pick it up then shoot

---

### Pattern 4: Complex Strategy (3 Balls)

```
Named Command: StartIntake      (Deploy arm)
Wait: 1.5 seconds               (Pick up first ball)
Named Command: StopIntake       (Retract arm)

Path: HubLocation               (Drive to hub)
Named Command: Shoot_Mid        (Shoot ball 1)

Path: SecondBallLocation        (Drive to second ball)
Named Command: StartIntake      (Deploy arm)
Wait: 1.5 seconds               (Pick up second ball)
Named Command: StopIntake       (Retract arm)

Path: HubLocation               (Drive back to hub)
Named Command: Shoot_Mid        (Shoot ball 2)

Path: ThirdBallLocation         (Drive to third ball)
Named Command: StartIntake      (Deploy arm)
Wait: 1.5 seconds               (Pick up third ball)
Named Command: StopIntake       (Retract arm)

Path: HubLocation               (Drive back to hub)
Named Command: Shoot_Mid        (Shoot ball 3)

Named Command: IdleFlywheel     (Stop spinning)
```

---

### Pattern 5: Shoot and Climb

```
Named Command: Shoot_Close          (Shoot close range)
Named Command: IdleFlywheel         (Stop flywheel)
Path: ClimberLocation               (Drive to climber)
Named Command: AutoClimbLevel1      (Auto climb)
```

---

## Part 6: Available Commands Reference

### Shooting

```
Shoot_Close   → 1.5m  | 2440 RPM | Auto spin, wait, feed, stop
Shoot_Mid     → 2.5m  | 2610 RPM | (same sequence)
Shoot_Far     → 4.0m  | 3000 RPM | (same sequence)
Shoot_VFar    → 5.5m  | 3600 RPM | (same sequence)
IdleFlywheel  → Stops flywheel (END YOUR AUTO WITH THIS!)
```

### Intake

```
StartIntake   → Deploy arm + run all rollers
StopIntake    → Retract arm + stop rollers
Eject         → Reverse all motors (push balls out)
```

### Other

```
AutoClimbLevel1      → Auto climb to Level 1 rung (3s timeout)
TurretAutoAim        → Aim turret using odometry + vision (5s timeout)
TurretAprilTagAim    → Aim turret using camera only (5s timeout)
StartFeeder          → Run feeder manually (usually not needed)
StopFeeder           → Stop feeder (usually not needed)
```

---

## Part 7: Creating Paths (Quick Overview)

### Before You Can Use a Path in Auto

You need to create the path first in PathPlanner:

1. **PathPlanner** → **Path** tab
2. **Create New Path**
3. **Mark start and end points** on field
4. **Draw a curve** (PathPlanner auto-generates smooth path)
5. **Save** (auto-saves to `src/main/deploy/pathplanner/paths/`)
6. **Use that path name** in your auto

### Available Paths

You already have:
- `Position1ToHub2.5meters` ← Use this one!
- `Pose2ToHub1.5Meters` ← Or this one!
- `Pose2ShootToClimb`
- `Position1ToHubMoveToPlayer`
- `Position1ToHubPlayerToHub`

**Use these existing paths as templates if you need more!**

---

## Part 8: Deploying and Testing

### Deploy Your Auto

```bash
cd C:\Users\Admin\Desktop\Robot2026Code
./gradlew deploy
```

**This sends your code + autos to the robot**

### Test Your Auto

1. **Connect robot to Driver Station**
2. **Open Elastic** (or Shuffleboard)
3. **Find "Auto Chooser"** dashboard
4. **Select your auto** from dropdown
5. **Robot is in AUTO mode** (not TELEOP)
6. **Press ENABLE** on Driver Station
7. **Watch robot execute!**

### Debug

If it doesn't work:

| Problem | Debug Steps |
|---------|-------------|
| Auto doesn't appear in chooser | 1. Save file as `.auto`<br>2. File in `autos/` folder<br>3. Run `./gradlew deploy`<br>4. Restart Elastic |
| Robot doesn't move | 1. Check path name is EXACT<br>2. Path file exists in `paths/`<br>3. Check odometry on dashboard |
| Shooter doesn't spin | 1. Check `Shoot_*` name is EXACT<br>2. Try shooting in TELEOP first<br>3. Check flywheel current draw |
| Intake doesn't deploy | 1. Run intake in TELEOP to verify<br>2. Check arm isn't stuck<br>3. Verify `StartIntake` command works |
| Auto stops early | 1. Check for timeout (5s limit on some)<br>2. Add `Wait` if timing too tight<br>3. Check robot error logs |

---

## Part 9: Pro Tips

### Timing

- **Intake pickup**: 1-2 seconds per ball
- **Flywheel spinup**: Built into `Shoot_*` (up to 2 sec)
- **Feeder run**: Built into `Shoot_*` (~0.5 sec)
- **Climbing**: 3 seconds

### Distances

Pick the right `Shoot_*` for your distance:

```
Distance    RPM    Command
-------     ---    --------
1.5 m      2440   Shoot_Close   ← Closest shot
2.5 m      2610   Shoot_Mid     ← Medium
4.0 m      3000   Shoot_Far     ← Far
5.5 m      3600   Shoot_VFar    ← Farthest
```

### Parallel (Advanced)

Want robot to aim turret while driving?

Use **"Parallel"** command type:
```
Parallel:
  ├─ Path: DriveToHub
  └─ TurretAutoAim
Then:
  Shoot_Mid
  IdleFlywheel
```

This makes turret aim **while** the robot drives!

---

## Part 10: Examples

### Example 1: Copy "Pos1ToShoot"

You already have this one working:
1. **Open** `Pos1ToShoot.auto` in Elastic
2. **It does:** Drive 2.5m → Shoot Mid → Idle
3. **Use as template** for similar routines

### Example 2: Quick Test

1. Create auto `TestQuick`
2. Add: `Shoot_Mid`
3. Add: `IdleFlywheel`
4. Save & deploy
5. Should just shoot!

### Example 3: Move & Shoot

1. Create auto `MoveTest`
2. Add: Path → `Position1ToHub2.5meters`
3. Add: `Shoot_Mid`
4. Add: `IdleFlywheel`
5. Save & deploy
6. Should move back then shoot!

---

## Summary

✅ **Paths** = Movement (Point A → Point B)
✅ **Named Commands** = Actions (Shoot, Intake, Climb)
✅ **Sequential** = Happens in order
✅ **Deploy** = Send to robot
✅ **Test** = Enable in auto mode

You're ready! Start building! 🚀

---

## Final Checklist

- [ ] PathPlanner installed
- [ ] Can access `src/main/deploy/pathplanner/`
- [ ] Read through a sample auto (Pos1ToShoot)
- [ ] Created first test auto
- [ ] Deployed code
- [ ] Selected auto in Elastic
- [ ] Pressed ENABLE
- [ ] Watched robot move!

If all checked ✓ → You're awesome! 🎉

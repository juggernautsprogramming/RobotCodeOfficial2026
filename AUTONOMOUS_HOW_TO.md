# How to Make Your Robot Move and Shoot Using PathPlanner

**TL;DR: Build autos by combining Paths (driving) + Named Commands (actions) in PathPlanner, then deploy.**

---

## Three Ways to Learn (Pick One)

### ⚡ I'm in a hurry (5 minutes)
1. Read: **AUTONOMOUS_QUICK_REFERENCE.md**
2. Open PathPlanner
3. Copy one of the test autos
4. Deploy
5. Done!

### 🎨 I learn better visually (20 minutes)
1. Read: **AUTONOMOUS_VISUAL_GUIDE.md**
2. Follow Part 1-4 step by step
3. Create your first auto
4. Deploy and test

### 📖 I want to understand everything (1 hour)
1. Read: **AUTONOMOUS_GUIDE.md**
2. Understand the structure
3. Study the examples
4. Read: **AUTONOMOUS_JSON_REFERENCE.md**
5. Build complex routines

---

## The 30-Second Version

An autonomous routine has three parts:

### 1. **Path** (Robot drives) 🛣️
```json
{
  "type": "path",
  "data": {
    "pathName": "Position1ToHub2.5meters"
  }
}
```

### 2. **Named Command** (Robot does action) ⚙️
```json
{
  "type": "named",
  "data": {
    "name": "Shoot_Mid"
  }
}
```

### 3. **Wait** (Just pause) ⏱️
```json
{
  "type": "wait",
  "data": {
    "waitTime": 1.5
  }
}
```

Combine them in **sequential** order = **AUTO!**

---

## Working Example: Drive Back & Shoot

This is a complete, ready-to-use autonomous routine:

```json
{
  "version": "2025.0",
  "command": {
    "type": "sequential",
    "data": {
      "commands": [
        {
          "type": "path",
          "data": {
            "pathName": "Position1ToHub2.5meters"
          }
        },
        {
          "type": "named",
          "data": {
            "name": "Shoot_Mid"
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
  "resetOdom": true,
  "folder": null,
  "choreoAuto": false
}
```

**What it does:**
1. Drives backward 2.5 meters
2. Spins up flywheel to 2610 RPM
3. Waits until RPM is on target (≤2 sec)
4. Runs feeder for ~0.5 sec
5. Stops flywheel
6. Done! ✓

---

## Step-by-Step: Build Your First Auto

### Step 1: Open PathPlanner

```
File → Open
Navigate to: C:\Users\Admin\Desktop\Robot2026Code\src\main\deploy\pathplanner
```

### Step 2: Click "Auto" Tab

At the bottom of the window, you'll see tabs: "Path", "Auto", "Autos"
Click **"Auto"**

### Step 3: Create New Auto

Click **"+"** button or **"Create"**
Name it: `MyFirstAuto`

### Step 4: Add Commands

For each command, click **"Add Command"** or **"+"**:

**Command 1 - Drive:**
- Type: **Path**
- Select: **"Position1ToHub2.5meters"**

**Command 2 - Shoot:**
- Type: **Named Command**
- Select: **"Shoot_Mid"**

**Command 3 - Stop:**
- Type: **Named Command**
- Select: **"IdleFlywheel"**

### Step 5: Save

Press **Ctrl+S** (or File → Save)

File is auto-saved to:
```
src/main/deploy/pathplanner/autos/MyFirstAuto.auto
```

### Step 6: Deploy

Open terminal in project folder:
```bash
./gradlew deploy
```

### Step 7: Test

In Elastic/Shuffleboard:
1. Find **"Auto Chooser"** dashboard
2. Select **"MyFirstAuto"**
3. Set robot to **AUTO mode** (not TELEOP)
4. Press **ENABLE** on Driver Station
5. Watch! 🤖

---

## Available Commands

### Shooting Commands (Auto Sequence)

Each does: spin up → wait for RPM → feed → stop

| Command | Distance | RPM |
|---------|----------|-----|
| `Shoot_Close` | 1.5 m | 2440 |
| `Shoot_Mid` | 2.5 m | 2610 |
| `Shoot_Far` | 4.0 m | 3000 |
| `Shoot_VFar` | 5.5 m | 3600 |
| `IdleFlywheel` | — | (stops) |

### Intake Commands

| Command | Does |
|---------|------|
| `StartIntake` | Deploy arm + run all rollers |
| `StopIntake` | Retract arm + stop rollers |
| `Eject` | Reverse all motors |

### Other Commands

| Command | Does |
|---------|------|
| `AutoClimbLevel1` | Auto climb to rung (3s timeout) |
| `TurretAutoAim` | Aim using odometry + vision (5s timeout) |
| `TurretAprilTagAim` | Aim using camera only (5s timeout) |

---

## Common Patterns

### Pattern 1: Just Shoot
```
Shoot_Mid
IdleFlywheel
```
**Use when:** Already at optimal distance, just need to shoot

### Pattern 2: Move Then Shoot
```
Path: Position1ToHub2.5meters
Shoot_Mid
IdleFlywheel
```
**Use when:** Need to drive to shooting position first

### Pattern 3: Collect & Shoot
```
StartIntake
Wait: 1.5 seconds
StopIntake
Shoot_Mid
IdleFlywheel
```
**Use when:** Ball is nearby, pick it up then shoot

### Pattern 4: Multi-Ball
```
StartIntake
Wait: 1.5
StopIntake
Path: HubLocation
Shoot_Mid

Path: SecondBall
StartIntake
Wait: 1.5
StopIntake
Path: HubLocation
Shoot_Mid

IdleFlywheel
```
**Use when:** Collecting multiple balls

### Pattern 5: Shoot & Climb
```
Shoot_Close
IdleFlywheel
Path: ToClimber
AutoClimbLevel1
```
**Use when:** Want to climb after shooting

---

## What You Already Have

### Test Auto Files
We created three example autos for you:

1. **TestSimpleShoot.auto** — Just shoots (no movement)
2. **TestMoveAndShoot.auto** — Moves back 2.5m then shoots
3. **TestIntakeAndShoot.auto** — Intake for 2s then shoots

**Use these to test!**

### Existing Auto Files
You already have working autos:

1. **Pos1ToShoot.auto** — Drive 2.5m back → Shoot Mid
2. **Pose2ShootToClimb.auto** — Drive 1.5m → Shoot Close → Drive to climber

**Copy these as templates!**

### Available Paths
You have existing paths ready to use:

- `Position1ToHub2.5meters` ← Good for backing up
- `Pose2ToHub1.5Meters` ← Close range
- `Pose2ShootToClimb` ← Leads to climber
- `Position1ToHubMoveToPlayer` ← Go grab ball
- `Position1ToHubPlayerToHub` ← Return to hub

---

## Troubleshooting

### Auto doesn't appear in chooser
- [ ] File saved as `.auto`?
- [ ] In `autos/` folder?
- [ ] Deployed with `./gradlew deploy`?
- [ ] Restarted Elastic?

### Robot doesn't move
- [ ] Path name spelled exactly right?
- [ ] Path file exists in `paths/` folder?
- [ ] Robot odometry showing movement on dashboard?

### Shooter doesn't spin
- [ ] Command name spelled exactly right?
- [ ] Works in TELEOP mode?
- [ ] Check current draw on dashboard?

### Auto stops early
- [ ] Check for 5s timeout on some commands
- [ ] Is timing too tight? Add a Wait
- [ ] Check robot console for errors

---

## Creating More Paths

If you need paths we don't have:

1. **PathPlanner** → **Path** tab
2. **Create New Path**
3. **Mark start point** on field
4. **Mark end point**
5. **Curve appears** (auto-generated)
6. **Save** (auto-saves to `paths/` folder)
7. **Use in auto** by name!

---

## File Structure

```
Robot2026Code/
├── src/main/deploy/pathplanner/
│   ├── autos/
│   │   ├── Pos1ToShoot.auto           ← Example
│   │   ├── Pose2ShootToClimb.auto     ← Example
│   │   ├── TestSimpleShoot.auto       ← NEW (test)
│   │   ├── TestMoveAndShoot.auto      ← NEW (test)
│   │   └── TestIntakeAndShoot.auto    ← NEW (test)
│   ├── paths/
│   │   ├── Position1ToHub2.5meters.path
│   │   ├── Pose2ToHub1.5Meters.path
│   │   └── ... (others)
│   ├── settings.json
│   └── navgrid.json
├── AUTONOMOUS_START_HERE.md           ← Overview
├── AUTONOMOUS_QUICK_REFERENCE.md      ← Cheat sheet
├── AUTONOMOUS_VISUAL_GUIDE.md         ← Step-by-step with visuals
├── AUTONOMOUS_GUIDE.md                ← Detailed guide
├── AUTONOMOUS_JSON_REFERENCE.md       ← JSON format details
└── README.md                          ← Command reference
```

---

## Key Takeaways

✅ **Paths** = Movement (Point A → Point B)
✅ **Named Commands** = Actions (Shoot, Intake, Climb)
✅ **Sequential** = Happens in order
✅ **Deploy** = `./gradlew deploy`
✅ **Test** = Select in Elastic, press ENABLE

✅ **Names are case-sensitive!**
✅ **Always end with `IdleFlywheel`!**
✅ **Paths must exist!**

---

## Next Steps

1. **Pick a guide above** (Quick, Visual, or Detailed)
2. **Follow the steps**
3. **Create your first auto**
4. **Deploy and test**
5. **Iterate based on results**

---

## One More Thing

### Robot Already Has Everything Set Up!

✅ All commands registered
✅ All paths created
✅ All autos configured
✅ Example files provided
✅ Documentation written

**You just need to:**
1. Open PathPlanner
2. Create sequence
3. Deploy
4. Test!

---

## Good Luck! 🚀

You've got this! Start with one of the guides above and build your first auto.

**It'll work. Trust us.** 💪

---

## Questions?

- **What commands can I use?** → README.md or AUTONOMOUS_QUICK_REFERENCE.md
- **How do I create a path?** → AUTONOMOUS_VISUAL_GUIDE.md Part 7
- **I don't understand JSON** → AUTONOMOUS_JSON_REFERENCE.md
- **I want step-by-step** → AUTONOMOUS_VISUAL_GUIDE.md
- **I want complete explanation** → AUTONOMOUS_GUIDE.md

Pick a document and start reading! 📚

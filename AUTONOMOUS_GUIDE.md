# PathPlanner Autonomous Routine Guide

## How to Create Move-and-Shoot Routines

Your robot already has all the commands registered for autonomous sequences. Here's how to build routines using PathPlanner.

---

## Step 1: Basic Structure

Every autonomous routine is a **sequential** command that combines:
1. **Path** — Robot drives to a location
2. **Named Command** — Robot performs an action (spin up, shoot, etc.)
3. **Path** — Robot drives to next location (optional)
4. **Repeat** — More paths and commands as needed

---

## Step 2: Available Commands

### Movement
- **Paths** — Drive from one point to another (created in PathPlanner)

### Shooting
- `Shoot_Close` (1.5 m, 2440 RPM) — Complete sequence: spin up → wait → feed → stop
- `Shoot_Mid` (2.5 m, 2610 RPM)
- `Shoot_Far` (4.0 m, 3000 RPM)
- `Shoot_VFar` (5.5 m, 3600 RPM)
- `IdleFlywheel` — Stop the flywheel (use at the end)

### Intake
- `StartIntake` — Deploy arm + run all rollers
- `StopIntake` — Retract arm + stop rollers
- `Eject` — Reverse intake to push balls out

### Feeder
- `StartFeeder` — Run feeder (for manual control)
- `StopFeeder` — Stop feeder

### Turret (Advanced)
- `TurretAutoAim` — Use odometry + vision to aim (5s timeout)
- `TurretAprilTagAim` — Use only camera yaw to aim (5s timeout)

---

## Step 3: Example Routines

### Example 1: Simple "Drive Back & Shoot"
**Scenario:** Start near the hub, back up 2.5 meters, then shoot.

**Structure:**
1. Drive backward 2.5 meters (using path "Position1ToHub2.5meters")
2. Shoot at mid range (`Shoot_Mid`)
3. Stop flywheel (`IdleFlywheel`)

**JSON:**
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

---

### Example 2: "Pickup → Shoot → Pickup → Shoot"
**Scenario:** Collect a ball, shoot, collect another ball, shoot again.

**Structure:**
1. Drive to first ball location
2. Start intake (`StartIntake`)
3. Wait 1 second for ball to be collected
4. Stop intake (`StopIntake`)
5. Drive to shooting position
6. Shoot (`Shoot_Mid`)
7. Drive to second ball
8. Start intake
9. Wait 1 second
10. Stop intake
11. Drive back to hub
12. Shoot again (`Shoot_Mid`)
13. Stop flywheel (`IdleFlywheel`)

**JSON:**
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
            "pathName": "PathToFirstBall"
          }
        },
        {
          "type": "named",
          "data": {
            "name": "StartIntake"
          }
        },
        {
          "type": "wait",
          "data": {
            "waitTime": 1.0
          }
        },
        {
          "type": "named",
          "data": {
            "name": "StopIntake"
          }
        },
        {
          "type": "path",
          "data": {
            "pathName": "BallToHub"
          }
        },
        {
          "type": "named",
          "data": {
            "name": "Shoot_Mid"
          }
        },
        {
          "type": "path",
          "data": {
            "pathName": "HubToSecondBall"
          }
        },
        {
          "type": "named",
          "data": {
            "name": "StartIntake"
          }
        },
        {
          "type": "wait",
          "data": {
            "waitTime": 1.0
          }
        },
        {
          "type": "named",
          "data": {
            "name": "StopIntake"
          }
        },
        {
          "type": "path",
          "data": {
            "pathName": "BallToHub"
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

---

### Example 3: "Shoot + Climb"
**Scenario:** Shoot from current position, then drive to climber.

**Structure:**
1. Shoot (`Shoot_Close`)
2. Idle flywheel (`IdleFlywheel`)
3. Drive to climber position
4. Auto climb (`AutoClimbLevel1`)

**JSON:**
```json
{
  "version": "2025.0",
  "command": {
    "type": "sequential",
    "data": {
      "commands": [
        {
          "type": "named",
          "data": {
            "name": "Shoot_Close"
          }
        },
        {
          "type": "named",
          "data": {
            "name": "IdleFlywheel"
          }
        },
        {
          "type": "path",
          "data": {
            "pathName": "HubToClimber"
          }
        },
        {
          "type": "named",
          "data": {
            "name": "AutoClimbLevel1"
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

---

## Step 4: Creating Your Auto Files

### In PathPlanner Desktop App:

1. **Open PathPlanner** → Click "Auto" tab at the bottom
2. **Create New Auto** → Give it a name (e.g., "MyFirstAuto")
3. **Build Your Sequence:**
   - Click `+` button to add a command
   - Choose between:
     - **Path** — Select from existing paths (Position1ToHub2.5meters, etc.)
     - **Named Command** — Select from list (Shoot_Mid, Shoot_Close, etc.)
     - **Wait** — Add delay in seconds
     - **Parallel** — Run multiple commands at once
   
4. **Set resetOdom** to `true` (top right) — this zeros the odometry at auto start
5. **Save** — Saves to your `src/main/deploy/pathplanner/autos/` folder

### File Structure:
```
src/main/deploy/pathplanner/
├── autos/
│   ├── Pos1ToShoot.auto
│   ├── Pose2ShootToClimb.auto
│   └── MyFirstAuto.auto          ← Your new file
├── paths/
│   ├── Position1ToHub2.5meters.path
│   └── ... (other paths)
└── settings.json
```

---

## Step 5: Testing Your Auto

1. **Deploy code to robot** (run `./gradlew deploy`)
2. **Open Elastic/Shuffleboard** → "Auto Chooser" tab
3. **Select your auto** from the dropdown
4. **Enable the robot in auto mode** on Driver Station
5. **Watch it execute!**

---

## Step 6: Common Patterns

### Pattern 1: Shoot Multiple Times from Different Distances
```
Drive to Position 1 → Shoot_Close → 
Drive to Position 2 → Shoot_Mid → 
Drive to Position 3 → Shoot_Far → 
IdleFlywheel
```

### Pattern 2: Collect and Shoot Loop
```
[Loop: Drive to Ball → StartIntake → Wait → StopIntake → Drive to Hub → Shoot_*] → 
IdleFlywheel
```

### Pattern 3: Aim While Moving (Advanced)
```
Parallel: [Drive Path] + [TurretAutoAim] →
Shoot_Mid →
IdleFlywheel
```
(Use `Parallel` command type to aim while driving)

---

## Step 7: Tuning Tips

### Timing
- **Intake collection**: 1-1.5 seconds per ball
- **Flywheel spin-up**: Built into `Shoot_*` commands (up to 2s)
- **Feed time**: Built into `Shoot_*` commands (~0.5s)

### Distance-Based Shooting
- **1.5m** → `Shoot_Close` (2440 RPM)
- **2.5m** → `Shoot_Mid` (2610 RPM)
- **4.0m** → `Shoot_Far` (3000 RPM)
- **5.5m** → `Shoot_VFar` (3600 RPM)

### Paths
- Make sure paths start from your robot's actual starting position
- Paths should end where you want to shoot or pick up the next ball
- Use "Reverse" toggle in PathPlanner if needed

---

## Step 8: Example: Create Your First Auto Right Now

1. In PathPlanner, create a new Auto called "TestShoot"
2. Add command: Path → "Position1ToHub2.5meters"
3. Add command: Named Command → "Shoot_Mid"
4. Add command: Named Command → "IdleFlywheel"
5. Save
6. Deploy and test!

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Auto doesn't appear in chooser | Did you save in `autos/` folder? Redeploy code. |
| Robot doesn't move | Check that path exists in `paths/` folder. |
| Robot moves but doesn't shoot | Make sure `Shoot_*` command is spelled correctly (case-sensitive). |
| Shooter doesn't spin up | Check that flywheel is enabled in teleop first. |
| Turret doesn't aim | Make sure turret is zeroed before auto (calibrate in teleop). |

---

## Next Steps

- **Autonomous Strategy Meeting**: Discuss which ball collection patterns maximize points
- **Path Creation**: Use PathPlanner to create paths for your specific strategy
- **Testing**: Test each segment separately before combining
- **Tuning**: Adjust timings and commands based on real-world performance

Good luck! 🚀

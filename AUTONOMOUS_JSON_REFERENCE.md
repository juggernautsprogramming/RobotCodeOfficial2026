# JSON Structure Reference

If you're editing `.auto` files directly, here's what everything means.

---

## Basic "Move and Shoot" Structure

```json
{
  "version": "2025.0",              // Keep this - PathPlanner format version
  "command": {
    "type": "sequential",            // Commands execute one after another
    "data": {
      "commands": [                  // Array of commands
        
        // COMMAND 1: Drive
        {
          "type": "path",            // This is a path (movement)
          "data": {
            "pathName": "Position1ToHub2.5meters"  // Name MUST match exactly!
          }
        },
        
        // COMMAND 2: Shoot
        {
          "type": "named",           // This is a named command (action)
          "data": {
            "name": "Shoot_Mid"      // Name MUST match exactly and case-sensitive!
          }
        },
        
        // COMMAND 3: Stop Flywheel
        {
          "type": "named",
          "data": {
            "name": "IdleFlywheel"
          }
        }
        
      ]
    }
  },
  "resetOdom": true,                 // Reset odometry to (0,0) at start - KEEP TRUE!
  "folder": null,                    // Leave as null
  "choreoAuto": false                // Leave as false
}
```

---

## What Each Field Means

### Top Level

| Field | Meaning | Keep As |
|-------|---------|---------|
| `version` | PathPlanner format | `"2025.0"` |
| `command` | Main command object | `{ ... }` |
| `resetOdom` | Zero odometry at start? | `true` |
| `folder` | Organization folder | `null` |
| `choreoAuto` | Use Choreo format? | `false` |

### Command Type: "sequential"

`"type": "sequential"` means commands execute one after another (in order).

Other types exist but sequential is what you want for move-and-shoot.

### Each Command

```json
{
  "type": "path",                    // Type: "path" or "named"
  "data": {
    "pathName": "..."                // For path type
    // OR
    "name": "..."                    // For named type
  }
}
```

---

## Command Types

### Path Command (Robot Drives)

```json
{
  "type": "path",
  "data": {
    "pathName": "Position1ToHub2.5meters"
  }
}
```

**Available paths:**
- `Position1ToHub2.5meters`
- `Pose2ToHub1.5Meters`
- `Pose2ShootToClimb`
- `Position1ToHubMoveToPlayer`
- `Position1ToHubPlayerToHub`

(Or any path you create in PathPlanner)

### Named Command (Robot Does Action)

```json
{
  "type": "named",
  "data": {
    "name": "Shoot_Mid"
  }
}
```

**Available commands:**

**Shooting:**
- `Shoot_Close` (1.5m, 2440 RPM)
- `Shoot_Mid` (2.5m, 2610 RPM)
- `Shoot_Far` (4.0m, 3000 RPM)
- `Shoot_VFar` (5.5m, 3600 RPM)
- `IdleFlywheel` (stop spinning)

**Intake:**
- `StartIntake`
- `StopIntake`
- `Eject`

**Other:**
- `AutoClimbLevel1`
- `TurretAutoAim`
- `TurretAprilTagAim`

### Wait Command (Just Pause)

```json
{
  "type": "wait",
  "data": {
    "waitTime": 1.5
  }
}
```

Wait 1.5 seconds before next command.

---

## Example Routines

### Example 1: Just Shoot

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

### Example 2: Drive and Shoot

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

### Example 3: Intake, Wait, Shoot

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
            "name": "StartIntake"
          }
        },
        {
          "type": "wait",
          "data": {
            "waitTime": 1.5
          }
        },
        {
          "type": "named",
          "data": {
            "name": "StopIntake"
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

### Example 4: Multi-Ball Collection

```json
{
  "version": "2025.0",
  "command": {
    "type": "sequential",
    "data": {
      "commands": [
        // First ball
        {
          "type": "path",
          "data": {
            "pathName": "Position1ToHub2.5meters"
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
            "waitTime": 1.5
          }
        },
        {
          "type": "named",
          "data": {
            "name": "StopIntake"
          }
        },
        
        // Shoot first ball
        {
          "type": "named",
          "data": {
            "name": "Shoot_Mid"
          }
        },
        
        // Go get second ball
        {
          "type": "path",
          "data": {
            "pathName": "Position1ToHubMoveToPlayer"
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
            "waitTime": 1.5
          }
        },
        {
          "type": "named",
          "data": {
            "name": "StopIntake"
          }
        },
        
        // Shoot second ball
        {
          "type": "path",
          "data": {
            "pathName": "Position1ToHubPlayerToHub"
          }
        },
        {
          "type": "named",
          "data": {
            "name": "Shoot_Mid"
          }
        },
        
        // Stop
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

## Key Rules

✅ **ALWAYS include:**
- `"version": "2025.0"`
- `"type": "sequential"`
- `"resetOdom": true`

✅ **Names are CASE-SENSITIVE:**
- `Shoot_Mid` ✓ (correct)
- `shoot_mid` ✗ (wrong!)
- `Shoot_mid` ✗ (wrong!)

✅ **Paths must exist:**
- Use exact name from your `paths/` folder
- Case-sensitive!

✅ **Always end with flywheel command:**
- Either `Shoot_*` (which includes stop)
- Or `IdleFlywheel` (explicit stop)

❌ **Don't change:**
- `"folder": null`
- `"choreoAuto": false`

---

## Valid Path Names

Check your `src/main/deploy/pathplanner/paths/` folder:

- `Position1ToHub2.5meters`
- `Pose2ToHub1.5Meters`
- `Pose2ShootToClimb`
- `Position1ToHubMoveToPlayer`
- `Position1ToHubPlayerToHub`

Use exact name, exact case!

---

## Valid Command Names

Check `README.md` or see list in AUTONOMOUS_QUICK_REFERENCE.md

**Shooting:**
- `Shoot_Close`, `Shoot_Mid`, `Shoot_Far`, `Shoot_VFar`
- `IdleFlywheel`

**Intake:**
- `StartIntake`, `StopIntake`, `Eject`

**Other:**
- `AutoClimbLevel1`, `TurretAutoAim`, `TurretAprilTagAim`

Use exact name, exact case!

---

## Testing Your JSON

If you edit JSON directly:
1. **Check syntax** - JSON must be valid (use online validator)
2. **Save** as `.auto` file
3. **Deploy**: `./gradlew deploy`
4. **Check Elastic** → Auto Chooser
5. **Select and test** with ENABLE button

---

## Debugging

**Auto appears but doesn't work:**
- Check command names (case-sensitive!)
- Check path names (case-sensitive!)
- Look at robot errors in console

**Auto doesn't appear:**
- Check filename ends in `.auto`
- Check file in `autos/` folder
- Redeploy code

**Robot does wrong thing:**
- Check command sequence order
- Verify path exists
- Check for typos

---

## Pro Tips

### Minimize Typing
Use PathPlanner GUI instead of editing JSON manually!

### Copy-Paste
Copy one of the example routines above as a starting template.

### Validate
Use online JSON validator if you hand-edit:
```
https://jsonlint.com/
```

### Format
Keep proper indentation (2 spaces) for readability.

---

That's it! Use this as a reference when creating or editing autos. 🚀

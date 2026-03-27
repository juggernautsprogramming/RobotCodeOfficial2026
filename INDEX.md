# Autonomous Routines Documentation Index

## 🎯 You Asked: "How can I make it move and shoot using PathPlanner?"

**Answer: Combine Paths (movement) + Named Commands (actions) in PathPlanner autos.**

Everything is already set up. Here's what we've created for you:

---

## 📚 Documents Created (Read in This Order)

### 1. **START HERE** 👈 `AUTONOMOUS_HOW_TO.md`
- **Time:** 5-30 minutes depending on depth
- **What:** The complete answer in one document
- **Best for:** Getting started quickly
- **Includes:** 
  - Quick explanation
  - Working example (copy-paste ready!)
  - Step-by-step instructions
  - Command reference
  - Troubleshooting

### 2. **Quick Cheat Sheet** `AUTONOMOUS_QUICK_REFERENCE.md`
- **Time:** 2 minutes
- **What:** Commands, patterns, deployment
- **Best for:** Quick lookup while building
- **Includes:**
  - TL;DR quick steps
  - Command list
  - Simple patterns
  - Copy-paste examples

### 3. **Visual Step-by-Step** `AUTONOMOUS_VISUAL_GUIDE.md`
- **Time:** 20-30 minutes
- **What:** Detailed walkthrough with explanations
- **Best for:** Learning the concepts
- **Includes:**
  - 10 parts (Basics → Pro Tips)
  - Step-by-step with actual screenshots/descriptions
  - Examples explained
  - Path creation guide
  - Testing & debugging

### 4. **Comprehensive Guide** `AUTONOMOUS_GUIDE.md`
- **Time:** 45+ minutes
- **What:** Deep dive into autonomous
- **Best for:** Understanding everything
- **Includes:**
  - Structure explanation
  - 3 complete working examples
  - All available commands
  - JSON format explanation
  - Common patterns
  - Tuning tips

### 5. **JSON Reference** `AUTONOMOUS_JSON_REFERENCE.md`
- **Time:** 10-15 minutes
- **What:** JSON format and syntax
- **Best for:** Hand-editing `.auto` files
- **Includes:**
  - Field meanings
  - Valid command names
  - Example routines (JSON)
  - Debugging JSON

### 6. **Overview** `AUTONOMOUS_START_HERE.md`
- **Time:** 5 minutes
- **What:** High-level overview
- **Best for:** Understanding the big picture
- **Includes:**
  - File locations
  - What you can do
  - Command cheat sheet
  - Deployment steps

### 7. **Command Reference** `README.md`
- **Time:** 2 minutes
- **What:** All available commands
- **Best for:** Lookup
- **Includes:**
  - Every registered command
  - What it does
  - Usage notes

---

## 🎮 What You Already Have

### Test Auto Files (Ready to Use)
```
src/main/deploy/pathplanner/autos/
├── TestSimpleShoot.auto         → Just shoots (no movement)
├── TestMoveAndShoot.auto        → Moves 2.5m back then shoots
└── TestIntakeAndShoot.auto      → Intake 2s then shoots
```

### Existing Auto Files (Copy as Templates)
```
src/main/deploy/pathplanner/autos/
├── Pos1ToShoot.auto             → Drive 2.5m → Shoot Mid
└── Pose2ShootToClimb.auto       → Drive 1.5m → Shoot → Climb
```

### Available Paths (Use in Your Autos)
```
src/main/deploy/pathplanner/paths/
├── Position1ToHub2.5meters.path     → Back up 2.5m
├── Pose2ToHub1.5Meters.path        → Back up 1.5m
├── Pose2ShootToClimb.path
├── Position1ToHubMoveToPlayer.path
└── Position1ToHubPlayerToHub.path
```

### Registered Commands (Use in PathPlanner)
**Shooting:** `Shoot_Close`, `Shoot_Mid`, `Shoot_Far`, `Shoot_VFar`, `IdleFlywheel`
**Intake:** `StartIntake`, `StopIntake`, `Eject`
**Other:** `AutoClimbLevel1`, `TurretAutoAim`, `TurretAprilTagAim`

---

## ⚡ Quick Start (5 Minutes)

1. Open PathPlanner → Auto tab
2. Create new auto
3. Add command: Path → "Position1ToHub2.5meters"
4. Add command: Named Command → "Shoot_Mid"
5. Add command: Named Command → "IdleFlywheel"
6. Save
7. Deploy: `./gradlew deploy`
8. Test: Select in Elastic, press ENABLE

**Done!** Robot will drive back 2.5m then shoot. 🚀

---

## 📖 Recommended Reading Paths

### Path A: "I Just Want to Build an Auto"
1. Read: AUTONOMOUS_HOW_TO.md (30 min)
2. Open PathPlanner
3. Follow step-by-step instructions
4. Deploy and test

### Path B: "I Want to Understand First"
1. Read: AUTONOMOUS_VISUAL_GUIDE.md (20 min)
2. Read: AUTONOMOUS_JSON_REFERENCE.md (10 min)
3. Open PathPlanner
4. Follow step-by-step instructions
5. Deploy and test

### Path C: "I Want Everything in Detail"
1. Read: AUTONOMOUS_GUIDE.md (45 min)
2. Read: AUTONOMOUS_JSON_REFERENCE.md (10 min)
3. Study working examples
4. Create complex routines
5. Deploy and test

### Path D: "Just Show Me the Cheat Sheet"
1. Read: AUTONOMOUS_QUICK_REFERENCE.md (2 min)
2. Copy example from this file
3. Modify as needed
4. Deploy and test

---

## 🎯 Document Features at a Glance

| Document | Type | Time | Best For | Key Feature |
|----------|------|------|----------|-------------|
| HOW_TO | Summary | 5-30m | Quick answer | Complete in one doc |
| QUICK_REFERENCE | Cheat sheet | 2m | Lookup | Copy-paste patterns |
| VISUAL_GUIDE | Tutorial | 20m | Learning | Step-by-step with explanations |
| GUIDE | Deep dive | 45m | Understanding | Everything explained |
| JSON_REFERENCE | Reference | 10m | Hand-editing | Format details |
| START_HERE | Overview | 5m | Big picture | File locations, capabilities |
| README | Reference | 2m | Command lookup | All available commands |

---

## 🎓 Learning Progression

### Beginner
- Read AUTONOMOUS_QUICK_REFERENCE.md
- Copy one test auto
- Deploy
- Test

### Intermediate
- Read AUTONOMOUS_HOW_TO.md
- Create your own auto from scratch
- Test multiple versions
- Tune timings

### Advanced
- Read AUTONOMOUS_GUIDE.md
- Read AUTONOMOUS_JSON_REFERENCE.md
- Hand-edit JSON for complex routines
- Create multi-path strategies

---

## 🔧 The Basics (TL;DR)

### An Auto Has Three Parts

**1. Paths** (Robot Drives) 🛣️
- `Position1ToHub2.5meters` — Drive backward 2.5 meters
- `Pose2ToHub1.5Meters` — Drive backward 1.5 meters
- Others in `paths/` folder

**2. Named Commands** (Robot Does Action) ⚙️
- `Shoot_Mid` — Spin to 2610 RPM, wait, feed, stop
- `Shoot_Close`, `Shoot_Far`, `Shoot_VFar` — Different distances
- `StartIntake`, `StopIntake` — Deploy/retract arm
- `IdleFlywheel` — Stop spinning

**3. Waits** (Just Pause) ⏱️
- `Wait 1.5 seconds` — For ball to settle, etc.

### Sequential Execution
They happen **in order, one after another**

### Example
```
1. Drive Path "Position1ToHub2.5meters"
2. Execute Named Command "Shoot_Mid" 
3. Execute Named Command "IdleFlywheel"
Done!
```

---

## 🚀 Deployment Process

```bash
# In PowerShell/Terminal in project folder
./gradlew deploy

# This uploads:
# - Java code
# - All .auto files from src/main/deploy/pathplanner/autos/
# - All path files
# - Configuration
```

Then:
1. **Elastic** → "Auto Chooser" → Select your auto
2. **Driver Station** → Set to AUTO mode
3. **Press ENABLE** → Robot executes!

---

## ✅ Success Checklist

- [ ] Opened AUTONOMOUS_HOW_TO.md and read it
- [ ] Opened PathPlanner Desktop App
- [ ] Created a new auto
- [ ] Added path + shoot commands
- [ ] Saved the auto file
- [ ] Deployed with `./gradlew deploy`
- [ ] Selected auto in Elastic
- [ ] Pressed ENABLE button
- [ ] Watched robot move and shoot!

If all checked ✓ → **You're done!** 🎉

---

## 🐛 Quick Troubleshooting

| Problem | Fix |
|---------|-----|
| Auto doesn't appear in chooser | Redeploy: `./gradlew deploy` |
| Robot doesn't move | Check path name spelled right |
| Robot doesn't shoot | Check command name spelled right |
| Unsure what to do | Read AUTONOMOUS_HOW_TO.md |
| Want to understand JSON | Read AUTONOMOUS_JSON_REFERENCE.md |
| Want step-by-step | Read AUTONOMOUS_VISUAL_GUIDE.md |

---

## 📞 Finding Answers

**Q: "What commands can I use?"**
A: README.md or AUTONOMOUS_QUICK_REFERENCE.md

**Q: "How do I create a path?"**
A: AUTONOMOUS_VISUAL_GUIDE.md Part 7

**Q: "What does this JSON mean?"**
A: AUTONOMOUS_JSON_REFERENCE.md

**Q: "How do I build my first auto?"**
A: AUTONOMOUS_HOW_TO.md (follow steps)

**Q: "I want to understand everything"**
A: AUTONOMOUS_GUIDE.md

**Q: "I'm stuck"**
A: Read AUTONOMOUS_HOW_TO.md or AUTONOMOUS_VISUAL_GUIDE.md

---

## 🎉 Final Notes

✅ **Everything is set up** — All commands registered, all paths created, examples provided

✅ **You have test files** — Three example autos to experiment with

✅ **Documentation is comprehensive** — Seven documents covering quick-to-detailed

✅ **Robot is ready** — Just needs your autonomous strategy

✅ **You've got this!** — Pick a document and start building! 🚀

---

## Recommended First Steps

1. **Pick your learning style:**
   - Quick & dirty? → AUTONOMOUS_QUICK_REFERENCE.md
   - Step-by-step? → AUTONOMOUS_VISUAL_GUIDE.md
   - Everything? → AUTONOMOUS_GUIDE.md

2. **Follow the guide** (5-30 minutes)

3. **Open PathPlanner** and create your first auto

4. **Deploy**: `./gradlew deploy`

5. **Test** in Elastic with ENABLE button

6. **Iterate** based on results

---

## Good Luck! 

You're about to create your first autonomous routine! 🤖🚀

Pick a guide above and dive in! 📚

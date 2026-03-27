# Autonomous Routines: Complete Guide

Welcome! This folder contains everything you need to build and understand autonomous move-and-shoot routines for your robot.

---

## 📚 Documents in This Folder

### 1. **AUTONOMOUS_QUICK_REFERENCE.md** ⚡
**Start here if you're in a hurry!**
- TL;DR quick steps
- Command cheat sheet
- Simple patterns
- Deployment commands

### 2. **AUTONOMOUS_VISUAL_GUIDE.md** 🎨
**Start here if you like step-by-step visuals!**
- Part 1: Understand the basics
- Part 2: Robot's capabilities  
- Part 3-5: Creating your first auto
- Part 6: Command reference
- Part 7: Creating paths
- Part 8: Deploying and testing
- Pro tips and examples

### 3. **AUTONOMOUS_GUIDE.md** 📖
**Start here for comprehensive details!**
- Deep explanation of structure
- 3 complete working examples
- JSON format walkthrough
- Common patterns
- Tuning tips
- Troubleshooting

### 4. **README.md** 📋
**See all available PathPlanner named commands**
- All registered commands
- What each command does
- How to use them

---

## 🚀 Quick Start (30 seconds)

1. **Open PathPlanner Desktop App**
2. **Click "Auto" tab** at bottom
3. **Create new auto**:
   ```
   Add Command → Path → "Position1ToHub2.5meters"
   Add Command → Named Command → "Shoot_Mid"
   Add Command → Named Command → "IdleFlywheel"
   ```
4. **Save** (Ctrl+S)
5. **Deploy**: `./gradlew deploy`
6. **Test**: Select auto in Elastic, press ENABLE

**That's it! Robot will drive backward 2.5m then shoot.** 🤖

---

## 📁 File Locations

```
src/main/deploy/pathplanner/
├── autos/
│   ├── Pos1ToShoot.auto           ← Existing working example
│   ├── Pose2ShootToClimb.auto     ← Existing working example
│   ├── TestSimpleShoot.auto       ← New test example (just shoots)
│   ├── TestMoveAndShoot.auto      ← New test example (moves + shoots)
│   └── TestIntakeAndShoot.auto    ← New test example (intake + shoots)
├── paths/
│   ├── Position1ToHub2.5meters.path  ← Use in autos!
│   ├── Pose2ToHub1.5Meters.path      ← Use in autos!
│   └── ... (other paths)
└── settings.json
```

---

## 🎯 What You Can Do

### Basic (Start Here)
- [x] Just shoot from current position
- [x] Drive back and shoot
- [x] Shoot at multiple distances

### Intermediate
- [x] Pick up a ball then shoot
- [x] Multi-ball collection and shooting
- [x] Drive, shoot, and climb

### Advanced (If Needed)
- [x] Aim turret while driving
- [x] Blend odometry + vision
- [x] Complex multi-path strategies

---

## 🎮 How It Works

Your autonomous routine is built from **three simple pieces**:

### 🛣️ Paths
"Drive from here to there"
- You design in PathPlanner path editor
- Examples: `Position1ToHub2.5meters`, `HubToClimber`

### ⚙️ Named Commands
"Do this action"
- Already registered and ready to use
- Examples: `Shoot_Mid`, `StartIntake`, `IdleFlywheel`

### ⏱️ Waits
"Just pause for a moment"
- Let the ball settle, etc.
- Example: Wait 1.5 seconds

### 🔗 Sequential
They execute **in order, one after another**

**Example flow:**
```
1. Drive backward 2.5m (Path)
   ↓
2. Spin flywheel to 2610 RPM (Named Command: Shoot_Mid)
   ↓
3. Wait until at speed (≤2 sec)
   ↓
4. Run feeder for ~0.5 sec
   ↓
5. Stop flywheel (Named Command: IdleFlywheel)
   ↓
Done! ✓
```

---

## 📋 Command Cheat Sheet

### Shooting (Just Pick One)
```
Shoot_Close   → 1.5 m, 2440 RPM  (auto spin → wait → feed → stop)
Shoot_Mid     → 2.5 m, 2610 RPM
Shoot_Far     → 4.0 m, 3000 RPM
Shoot_VFar    → 5.5 m, 3600 RPM
IdleFlywheel  → Stop flywheel (ALWAYS use at end!)
```

### Intake
```
StartIntake   → Deploy arm + run all rollers
StopIntake    → Retract arm + stop rollers
Eject         → Reverse intake (push balls out)
```

### Other
```
AutoClimbLevel1      → Auto climb to rung
TurretAutoAim        → Aim using odometry + vision
TurretAprilTagAim    → Aim using camera only
```

---

## 🔧 Creating Your First Auto

### Option A: In PathPlanner GUI (Easiest)
1. Open PathPlanner Desktop
2. Auto tab → Create new → Add commands → Save
3. Done!

### Option B: In VS Code (For Editing Existing)
1. Open `src/main/deploy/pathplanner/autos/MyAuto.auto`
2. Edit JSON directly
3. Save
4. Redeploy

---

## 🧪 Test Autos (Use These!)

We've created three test autos for you:

### TestSimpleShoot.auto
- **What it does:** Just shoots
- **Sequence:** Spin up → Wait → Feed → Stop
- **Use case:** Verify shooting works
- **Expected:** Hear motor spin and feeder click

### TestMoveAndShoot.auto  
- **What it does:** Drive backward 2.5m, then shoot
- **Sequence:** Drive → Shoot Mid → Idle
- **Use case:** Verify movement + shooting together
- **Expected:** Robot backs up, then shoots

### TestIntakeAndShoot.auto
- **What it does:** Run intake for 2s, then shoot
- **Sequence:** StartIntake → Wait 2s → StopIntake → Shoot Mid → Idle
- **Use case:** Verify intake works in auto
- **Expected:** Arm deploys, waits, retracts, shoots

**Test each one individually first!**

---

## 🚀 Deployment

After creating/editing an auto:

```bash
cd C:\Users\Admin\Desktop\Robot2026Code
./gradlew deploy
```

This uploads:
- Your Java code
- All auto files from `src/main/deploy/pathplanner/`
- All path files
- All configuration

---

## ✅ Verification

In Elastic/Shuffleboard:
1. **Find "Auto Chooser"** dashboard
2. **See your auto in dropdown?** ✓ Deployment worked!
3. **Don't see it?** Check filename, folder, and redeploy

---

## 🐛 Troubleshooting

| Problem | Solution |
|---------|----------|
| Auto doesn't appear | File in `autos/` folder? Redeploy? Restart Elastic? |
| Robot doesn't move | Path name exact? Path file exists? Odometry zero? |
| Shooter doesn't spin | Command name exact? Works in teleop? Check current draw? |
| Intake doesn't deploy | Works in teleop first? Arm stuck? Power connected? |
| Auto stops halfway | Timeout? Timing too tight? Add `Wait` command? |

---

## 🎓 Learning Path

### Day 1: Understand
- [ ] Read AUTONOMOUS_QUICK_REFERENCE.md (5 min)
- [ ] Look at existing autos: Pos1ToShoot, Pose2ShootToClimb
- [ ] Understand the three pieces: Path, Named Command, Wait

### Day 2: Try One
- [ ] Create test auto using TestMoveAndShoot as template
- [ ] Deploy
- [ ] Test in robot

### Day 3: Build Your Strategy
- [ ] Plan which balls you'll collect
- [ ] Create paths in PathPlanner to them
- [ ] Build multi-command auto sequence

### Day 4: Iterate
- [ ] Test each segment
- [ ] Tune timings
- [ ] Optimize for maximum points

---

## 📞 Questions?

**"How do I create a path?"**
→ Read AUTONOMOUS_VISUAL_GUIDE.md Part 7

**"What commands are available?"**
→ Check README.md or AUTONOMOUS_GUIDE.md Step 2

**"How do I make it aim while driving?"**
→ AUTONOMOUS_GUIDE.md Step 6, Pattern 3 (Advanced)

**"I'm stuck, where do I start?"**
→ Read AUTONOMOUS_QUICK_REFERENCE.md (TL;DR)

---

## 🎉 You've Got This!

Everything is set up and ready to go. Pick a document above and start building! 

**Recommended order:**
1. AUTONOMOUS_QUICK_REFERENCE.md (2 min)
2. Look at TestMoveAndShoot.auto (1 min)
3. Create your first auto (5 min)
4. Deploy and test (5 min)
5. Iterate based on results

**Total time: ~15 minutes to your first working autonomous!** ⏱️

Good luck! 🚀

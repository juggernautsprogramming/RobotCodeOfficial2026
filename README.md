# Robot 2026 Code

## PathPlanner Named Commands

Type these names exactly (case-sensitive) into PathPlanner event markers.

---

### Turret

| Command | What it does | Notes |
|---|---|---|
| `TurretAutoAim` | Continuously aims turret using odometry + vision blend | Uses FireControlSolver; provides velocity compensation |
| `TurretAprilTagAim` | Aims turret purely from AprilTag camera yaw | No odometry; good when hub tag is visible |

---

### Shooting (Recommended)

| Command | What it does |
|---|---|
| `SnapAimAndShoot` | ⭐ **RECOMMENDED** — Aligns to hub tag, calculates RPM from distance, spins up, and shoots |

**How it works:**
1. Robot aligns to hub AprilTag
2. Measures distance to hub
3. Calculates optimal RPM from distance
4. Spins flywheel to calculated RPM
5. Waits for RPM on target
6. Feeds all balls
7. Complete!

---

### Shooting (Legacy - Fixed Distance)

| Command | Distance | RPM | Notes |
|---|---|---|---|
| `Shoot_Close` | 1.5 m | 2440 | DEPRECATED - Use SnapAimAndShoot instead |
| `Shoot_Mid` | 2.5 m | 2610 | DEPRECATED - Use SnapAimAndShoot instead |
| `Shoot_Far` | 4.0 m | 3000 | DEPRECATED - Use SnapAimAndShoot instead |
| `Shoot_VFar` | 5.5 m | 3600 | DEPRECATED - Use SnapAimAndShoot instead |

**Note:** These are kept for backwards compatibility but are not recommended. Use `SnapAimAndShoot` for distance-based auto RPM calculation.

---

### Intake

| Command | What it does |
|---|---|
| `StartIntake` | Deploys arm + spins all intake rollers |
| `StopIntake` | Retracts arm + stops all rollers |
| `Eject` | Reverses all intake motors to eject balls |

---

### Feeder

| Command | What it does |
|---|---|
| `StartFeeder` | Runs feeder roller at 5V |
| `StopFeeder` | Stops feeder roller |

---

### Flywheel Spin-Up Presets

Instant commands — flywheel stays at speed until `IdleFlywheel` is called.
Pair with `StartFeeder` / `StopFeeder` if you want manual feed control (usually not needed with `SnapAimAndShoot`).

| Command | Distance | RPM |
|---|---|---|
| `SpinUp_Close` | 1.5 m | 2440 |
| `SpinUp_Mid` | 2.5 m | 2610 |
| `SpinUp_Far` | 4.0 m | 3000 |
| `SpinUp_VFar` | 5.5 m | 3600 |
| `SpinUpFlywheel` | Optimal (auto-calculated) | — |
| `IdleFlywheel` | Stops flywheel | — |

---

### Climbing

| Command | What it does | Notes |
|---|---|---|
| `AutoClimbLevel1` | Climbs to Level 1 rung | 3s timeout; add at end of auto path |

---

### Autos

| Auto Name | Description |
|---|---|
| `CenterToShoot` | Start near hub center, drive back 1.5 m, shoot at mid range, idle flywheel |

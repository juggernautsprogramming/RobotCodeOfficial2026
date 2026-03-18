# Robot 2026 Code

## PathPlanner Named Commands

Type these names exactly (case-sensitive) into PathPlanner event markers.

---

### Intake

| Command | What it does |
|---|---|
| `StartIntake` | Deploys arm + spins all intake rollers |
| `StopIntake` | Retracts arm + stops all rollers |

---

### Feeder

| Command | What it does |
|---|---|
| `StartFeeder` | Runs feeder roller at 5V |
| `StopFeeder` | Stops feeder roller |

---

### Flywheel Spin-Up Presets

Instant commands — flywheel stays at speed until `IdleFlywheel` is called.
Pair with `StartFeeder` / `StopFeeder` if you want manual feed control.

| Command | Distance | RPM |
|---|---|---|
| `SpinUp_Close` | 1.5 m | 2440 |
| `SpinUp_Mid` | 2.5 m | 2610 |
| `SpinUp_Far` | 4.0 m | 3000 |
| `SpinUp_VFar` | 5.5 m | 3600 |
| `SpinUpFlywheel` | Optimal (auto-calculated) | — |
| `IdleFlywheel` | Stops flywheel | — |

---

### Full Shoot Sequences

Self-contained — automatically spins up flywheel, waits for target RPM (≤ 2 s), runs feeder for 0.5 s, then stops feeder.
Flywheel keeps spinning after; follow with `IdleFlywheel` at the end of your auto.

| Command | Distance | RPM |
|---|---|---|
| `Shoot_Close` | 1.5 m | 2440 |
| `Shoot_Mid` | 2.5 m | 2610 |
| `Shoot_Far` | 4.0 m | 3000 |
| `Shoot_VFar` | 5.5 m | 3600 |

---

### Autos

| Auto Name | Description |
|---|---|
| `CenterToShoot` | Start near hub center, drive back 1.5 m, shoot at mid range, idle flywheel |

# Climber Position Tuning Guide

## Overview
Your climber has two fixed positions (DOWN and UP) that are controlled by motor rotations. You need to find these two positions by manually jogging the climber with the joystick.

## Step-by-Step Tuning Process

### 1. Deploy the Code
- Compile and deploy the updated code to the robot
- Open SmartDashboard on your driver station

### 2. Find the DOWN Position (Resting)

**Setup:**
- Release the brake (if you can) or ensure the climber can be moved
- Get to the robot safely

**Procedure:**
1. On the **operator controller (port 1)**, **hold the RIGHT TRIGGER** to move the climber down
   - Power level: `-0.1` (gentle, slow movement)
   - The climber will move downward while you hold the trigger
   
2. Move it down to the **resting position** (hooks fully retracted, at rest)
3. **Release the RIGHT TRIGGER**
4. Look at **SmartDashboard** → **Climber/Position_rot**
5. **Write down this number** — this is your `DOWN_POSITION_ROTATIONS`

**Example:** If the dashboard shows `2.5`, then:
```java
public static final double DOWN_POSITION_ROTATIONS = 2.5;
```

### 3. Find the UP Position (Hooks Extended)

**Setup:**
- Use the same SmartDashboard display as before

**Procedure:**
1. On the **operator controller**, **hold the LEFT TRIGGER** to move the climber up
   - Power level: `0.4` (matches your testing dutycycle)
   - The climber will move upward while you hold the trigger
   
2. Move it up until the **hooks just touch the rung** (or reach your desired height)
3. **Release the LEFT TRIGGER**
4. Look at **SmartDashboard** → **Climber/Position_rot**
5. **Write down this number** — this is your `UP_POSITION_ROTATIONS`

**Example:** If the dashboard shows `8.3`, then:
```java
public static final double UP_POSITION_ROTATIONS = 8.3;
```

### 4. Update Constants.java

Open `src/main/java/frc/robot/Constants.java` and find the `ClimberConstants` section:

```java
public static final double DOWN_POSITION_ROTATIONS = 0.0; // ⚠ tune on robot
public static final double UP_POSITION_ROTATIONS = 0.0;   // ⚠ tune on robot
```

Replace the `0.0` values with your recorded numbers:

```java
public static final double DOWN_POSITION_ROTATIONS = 2.5;  // YOUR DOWN VALUE
public static final double UP_POSITION_ROTATIONS = 8.3;    // YOUR UP VALUE
```

### 5. Test the Automated Positions

1. **Recompile and deploy** the code
2. **Command the climber UP:**
   - Use a test command or manually drag it slightly, then release a trigger
   - It should automatically move to the UP position and stop
   - Watch `Climber/AtTarget` on SmartDashboard — it should show `true` when arrived
   
3. **Command the climber DOWN:**
   - Similar test for the DOWN position
   - Verify it reaches the exact same spot each time

### 6. Fine-Tune Tolerance (if needed)

If the climber overshoots or doesn't reach the position accurately, adjust `POSITION_TOLERANCE_ROTATIONS`:

```java
public static final double POSITION_TOLERANCE_ROTATIONS = 0.05; // Currently ≈ 18 degrees
```

- **Smaller value** = more precise but may oscillate
- **Larger value** = more forgiving but less precise

## Troubleshooting

**Climber doesn't move when I hold triggers:**
- Check that the brake isn't engaged
- Verify the motor power levels (0.4 for up, -0.1 for down)
- Check that the motor CAN IDs are correct (15 and 16)

**Climber reaches different positions each time:**
- Ensure `POSITION_TOLERANCE_ROTATIONS` isn't too large
- Check that `kP`, `kI`, `kD` gains are appropriate
- Verify there's no friction or mechanical binding

**Numbers seem wrong:**
- Reset the motor position to 0 if the climber was bumped
- Re-check by moving to the same physical position — the rotation value should be close

## Dashboard Values to Monitor

- **Climber/Position_rot** — Current position in rotations (use this for tuning!)
- **Climber/Position_deg** — Current position in degrees (FYI)
- **Climber/Target** — Which position you're targeting (DOWN or UP)
- **Climber/AtTarget** — Whether you've reached the target position

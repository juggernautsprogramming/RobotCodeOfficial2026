package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * ShiftStateTracker — reads the 2026 REBUILT game-specific message and
 * determines whether the robot's alliance hub is currently ACTIVE.
 *
 * <h3>Game message</h3>
 * FMS sends a single character ~3 seconds after auto ends:
 * <ul>
 *   <li>{@code "R"} — Red alliance hub is inactive in Shifts 1 &amp; 3</li>
 *   <li>{@code "B"} — Blue alliance hub is inactive in Shifts 1 &amp; 3</li>
 *   <li>{@code ""}  — Not yet received (treat as both active)</li>
 * </ul>
 * The alliance that scored <em>fewer</em> FUEL in auto is inactive first.
 *
 * <h3>Teleop shift windows</h3>
 * {@code DriverStation.getMatchTime()} counts DOWN from ~135 s:
 * <pre>
 *   ≥ 130 s  Transition — both hubs active
 *   129–105  Shift 1    — first inactive
 *   104–80   Shift 2    — other inactive
 *    79–55   Shift 3    — first inactive again
 *    54–30   Shift 4    — other inactive again
 *   ≤ 29 s   End Game   — both hubs active
 * </pre>
 */
public final class ShiftStateTracker {

    // Teleop match-time thresholds (counts DOWN from ~135 s)
    private static final double TRANSITION_END_S = 130.0;
    private static final double SHIFT1_END_S     = 105.0;
    private static final double SHIFT2_END_S     =  80.0;
    private static final double SHIFT3_END_S     =  55.0;
    private static final double END_GAME_START_S =  30.0;

    private ShiftStateTracker() {}

    /**
     * Returns {@code true} when the current alliance's hub is ACTIVE and can accept scoring.
     * Always returns {@code true} outside teleop, or before game data arrives.
     */
    public static boolean isMyHubActive() {
        if (!DriverStation.isTeleopEnabled()) return true;

        String msg = DriverStation.getGameSpecificMessage();
        if (msg == null || msg.isEmpty()) return true; // data not yet received

        Alliance mine = DriverStation.getAlliance().orElse(Alliance.Blue);
        // Which alliance's hub goes inactive in Shifts 1 and 3?
        boolean iAmOddShiftInactive = (msg.equals("R") && mine == Alliance.Red)
                                   || (msg.equals("B") && mine == Alliance.Blue);

        double t = DriverStation.getMatchTime();
        if (t >= TRANSITION_END_S) return true;               // Transition
        if (t >= SHIFT1_END_S)     return !iAmOddShiftInactive; // Shift 1
        if (t >= SHIFT2_END_S)     return  iAmOddShiftInactive; // Shift 2
        if (t >= SHIFT3_END_S)     return !iAmOddShiftInactive; // Shift 3
        if (t >= END_GAME_START_S) return  iAmOddShiftInactive; // Shift 4
        return true;                                            // End Game
    }

    /**
     * Publishes shift state to SmartDashboard.
     * Call from {@code Robot.teleopPeriodic()} once per loop.
     */
    public static void publishTelemetry() {
        boolean active = isMyHubActive();
        SmartDashboard.putBoolean("Shift/HubActive",    active);
        SmartDashboard.putString ("Shift/Status",       active ? "ACTIVE - SHOOT" : "INACTIVE - WAIT");
        SmartDashboard.putString ("Shift/GameMessage",  DriverStation.getGameSpecificMessage());
        SmartDashboard.putString ("Shift/CurrentShift", getCurrentShiftName());
    }

    private static String getCurrentShiftName() {
        if (!DriverStation.isTeleopEnabled()) return "—";
        double t = DriverStation.getMatchTime();
        if (t >= TRANSITION_END_S) return "Transition";
        if (t >= SHIFT1_END_S)     return "Shift 1";
        if (t >= SHIFT2_END_S)     return "Shift 2";
        if (t >= SHIFT3_END_S)     return "Shift 3";
        if (t >= END_GAME_START_S) return "Shift 4";
        return "End Game";
    }
}

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.ShooterConstants;

/**
 * ShotCalculator — ballistic solver, optimal-angle finder, and optimal-standoff
 * finder using the exact physics from {@code frc_v0_calculator.html}.
 *
 * <h2>Physics model (directly from the HTML calculator)</h2>
 * <pre>
 *   h   = h2 − h1                          (net height the ball must gain)
 *   v₀  = d × √( g / ( 2·cos²θ · (d·tanθ − h) ) )
 *   t   = d / (v₀ · cosθ)                  (flight time)
 *   vy_final = v₀·sinθ − g·t              (vertical velocity at target)
 *   entry_angle = atan2(−vy_final, v₀·cosθ)   (how steeply the ball falls into hub)
 * </pre>
 *
 * <h2>Optimal angle finder (per distance)</h2>
 * Sweeps every {@link ShooterConstants#ANGLE_SWEEP_STEP_DEG}° from
 * {@link ShooterConstants#MIN_LAUNCH_ANGLE_DEG} to
 * {@link ShooterConstants#MAX_LAUNCH_ANGLE_DEG} and scores each candidate using
 * the same weighted formula as the HTML Optimizer tab:
 * <pre>
 *   tScore   = max(0, 1 − t / 1.2)           (shorter flight = better)
 *   v0Score  = max(0, 1 − v0 / 20)           (lower exit speed = better)
 *   eaScore  = min(1, entryAngle / 65)        (steeper entry = better accuracy)
 *   rpmScore = max(0, 1 − motorRPM / 6000)   (lower RPM = better)
 *   score    = w[0]·tScore + w[1]·v0Score + w[2]·eaScore + w[3]·rpmScore
 * </pre>
 * Weights come from {@link ShooterConstants#SHOT_SCORE_WEIGHTS}.
 *
 * <h2>Optimal standoff distance</h2>
 * {@link #OPTIMAL_STANDOFF_M} is precomputed once at class load by sweeping
 * distances from {@link ShooterConstants#OPTIMAL_DIST_MIN_M} to
 * {@link ShooterConstants#OPTIMAL_DIST_MAX_M} and picking the distance whose
 * best angle achieves the highest score.  This answers the question:
 * <em>"Where should the robot stand to get the best possible shot?"</em>
 *
 * <p>All sweep work happens at JVM startup — zero runtime cost during the match.
 */
public final class ShotCalculator {

    // ── Physical constants (from the HTML calculator) ─────────────────────────
    private static final double G          = ShooterConstants.g;
    private static final double H1         = ShooterConstants.SHOOTER_EXIT_HEIGHT_METERS;
    private static final double H2         = ShooterConstants.HUB_TARGET_HEIGHT_METERS;
    private static final double DELTA_H    = H2 - H1;   // net height the ball must gain

    // ── Flywheel geometry (for RPM conversion, matching HTML RPM tab) ─────────
    private static final double WHEEL_DIAM_M = ShooterConstants.WHEEL_DIAMETER_INCHES * 0.0254;
    private static final double ETA          = ShooterConstants.EFFICIENCY;
    private static final double GEAR_RATIO   = ShooterConstants.SHOOTER_GEAR_RATIO;

    // ── Precomputed optimal standoff ──────────────────────────────────────────

    /**
     * The standoff distance (metres, hub centre → robot centre) that yields the
     * highest scoring shot given the robot's hardware.
     *
     * <p>Computed once at class load by sweeping
     * [{@link ShooterConstants#OPTIMAL_DIST_MIN_M},
     *  {@link ShooterConstants#OPTIMAL_DIST_MAX_M}] in
     *  {@link ShooterConstants#OPTIMAL_DIST_STEP_M} increments.
     * Use this in your drive commands instead of the hardcoded constant.
     */
    public static final double OPTIMAL_STANDOFF_M;

    /**
     * The shot parameters at {@link #OPTIMAL_STANDOFF_M}.
     * Contains the optimal hood angle, RPM, and entry angle for the best position.
     */
    public static final ShotResult OPTIMAL_SHOT;

    static {
        // Sweep distances and pick the one whose best angle gives the highest score.
        // All work is done at class load — no per-loop cost.
        double bestDist  = (ShooterConstants.OPTIMAL_DIST_MIN_M + ShooterConstants.OPTIMAL_DIST_MAX_M) / 2.0;
        double bestScore = -1.0;

        for (double d = ShooterConstants.OPTIMAL_DIST_MIN_M;
                   d <= ShooterConstants.OPTIMAL_DIST_MAX_M;
                   d += ShooterConstants.OPTIMAL_DIST_STEP_M) {
            ShotResult r = calculate(d);
            if (r.score() > bestScore) {
                bestScore = r.score();
                bestDist  = d;
            }
        }

        OPTIMAL_STANDOFF_M = bestDist;
        OPTIMAL_SHOT       = calculate(OPTIMAL_STANDOFF_M);

        // Log to console so the driver station shows the result at startup
        System.out.printf(
            "[ShotCalculator] Optimal standoff: %.2f m | hood: %.1f° | RPM: %.0f | entry: %.1f° | score: %.3f%n",
            OPTIMAL_STANDOFF_M,
            OPTIMAL_SHOT.hoodDeg(),
            OPTIMAL_SHOT.rpm(),
            OPTIMAL_SHOT.entryAngle(),
            OPTIMAL_SHOT.score());
    }

    private ShotCalculator() {} // static-only utility class

    // ── Public result record ───────────────────────────────────────────────────

    /**
     * @param rpm          Motor RPM setpoint — use this in {@code setFlywheelRPM()}.
     * @param hoodDeg      Optimal launch angle (degrees above horizontal).
     * @param v0           Ball exit speed (m/s).
     * @param flightTime   Estimated flight time (seconds).
     * @param entryAngle   Ball entry angle into the hub (degrees) — &gt;35° is good.
     * @param score        Weighted quality score [0, 1] from the HTML optimizer formula.
     */
    public record ShotResult(
            double rpm,
            double hoodDeg,
            double v0,
            double flightTime,
            double entryAngle,
            double score) {}

    // ── Primary API ───────────────────────────────────────────────────────────

    /**
     * Calculate the optimal shot parameters for a given distance.
     *
     * <p>Sweeps launch angles and scores each one, then returns the highest-scoring
     * valid angle and its derived RPM.
     *
     * @param distanceMeters Horizontal distance from shooter exit to hub centre (metres).
     * @return Best {@link ShotResult}, or a safe fallback (score=0) if no valid angle exists.
     */
    public static ShotResult calculate(double distanceMeters) {
        double d = MathUtil.clamp(distanceMeters, 0.5, 8.0);

        double bestScore = -1.0;
        double bestAngle = ShooterConstants.MIN_LAUNCH_ANGLE_DEG;
        Physics bestPhys = null;

        double step = ShooterConstants.ANGLE_SWEEP_STEP_DEG;
        for (double theta = ShooterConstants.MIN_LAUNCH_ANGLE_DEG;
                   theta <= ShooterConstants.MAX_LAUNCH_ANGLE_DEG;
                   theta += step) {

            Physics p = solvePhysics(d, theta);
            if (p == null) continue;                                        // ball cannot reach target
            if (p.entryAngle < ShooterConstants.MIN_ENTRY_ANGLE_DEG) continue; // too flat
            double motorRPM = toMotorRPM(p.v0);
            if (motorRPM > ShooterConstants.MAX_MOTOR_RPM) continue;       // flywheel too slow

            double score = scoreAngle(p, motorRPM);
            if (score > bestScore) {
                bestScore = score;
                bestAngle = theta;
                bestPhys  = p;
            }
        }

        if (bestPhys == null) {
            return new ShotResult(ShooterConstants.IDLE_RPM, 45.0, 0, 0, 0, 0.0);
        }

        return new ShotResult(
            toMotorRPM(bestPhys.v0),
            bestAngle,
            bestPhys.v0,
            bestPhys.flightTime,
            bestPhys.entryAngle,
            bestScore);
    }

    /**
     * Returns the precomputed shot parameters at the physics-optimal standoff distance.
     * Equivalent to {@code calculate(OPTIMAL_STANDOFF_M)} but zero-cost (already cached).
     */
    public static ShotResult calculateOptimal() {
        return OPTIMAL_SHOT;
    }

    /**
     * Sweep a range of distances and return the one whose best angle achieves the
     * highest overall score.  Used internally to populate {@link #OPTIMAL_STANDOFF_M}
     * but also callable at runtime if you want to re-evaluate with different bounds.
     *
     * @param minM   Nearest distance to consider (metres).
     * @param maxM   Farthest distance to consider (metres).
     * @param stepM  Distance step size (metres).
     * @return Optimal standoff distance (metres).
     */
    public static double findOptimalDistance(double minM, double maxM, double stepM) {
        double bestDist  = (minM + maxM) / 2.0;
        double bestScore = -1.0;
        for (double d = minM; d <= maxM; d += stepM) {
            ShotResult r = calculate(d);
            if (r.score() > bestScore) {
                bestScore = r.score();
                bestDist  = d;
            }
        }
        return bestDist;
    }

    // ── Direct physics query (for diagnostics) ────────────────────────────────

    /**
     * Compute physics for an explicit angle and distance.
     * Returns {@code null} if the ball cannot reach the target.
     *
     * @param distanceMeters Horizontal distance to hub (metres).
     * @param thetaDeg       Launch angle (degrees).
     */
    public static ShotResult calculateAtAngle(double distanceMeters, double thetaDeg) {
        Physics p = solvePhysics(distanceMeters, thetaDeg);
        if (p == null) return null;
        double mRPM  = toMotorRPM(p.v0);
        double score = scoreAngle(p, mRPM);
        return new ShotResult(mRPM, thetaDeg, p.v0, p.flightTime, p.entryAngle, score);
    }

    // ── Internal physics solver ───────────────────────────────────────────────

    /**
     * Exact formula from the HTML calculator:
     * <pre>
     *   denom = 2 · cos²θ · (d·tanθ − h)
     *   v₀    = d · √(g / denom)           [only valid when denom &gt; 0]
     *   t     = d / (v₀ · cosθ)
     *   vy_f  = v₀·sinθ − g·t
     *   ea    = atan2(−vy_f, v₀·cosθ)      [entry angle in degrees]
     * </pre>
     */
    private static Physics solvePhysics(double d, double thetaDeg) {
        double th    = Math.toRadians(thetaDeg);
        double cosT  = Math.cos(th);
        double sinT  = Math.sin(th);
        double denom = 2.0 * cosT * cosT * (d * Math.tan(th) - DELTA_H);

        if (denom <= 0.0 || d <= 0.0) return null;

        double v0    = d * Math.sqrt(G / denom);
        double t     = d / (v0 * cosT);
        double vyFin = v0 * sinT - G * t;
        double ea    = Math.toDegrees(Math.atan2(-vyFin, v0 * cosT));

        return new Physics(v0, t, ea);
    }

    /**
     * Scoring function — identical to the HTML {@code scoreAngle()} function.
     * Weights from {@link ShooterConstants#SHOT_SCORE_WEIGHTS}.
     */
    private static double scoreAngle(Physics p, double motorRPM) {
        double tScore   = Math.max(0.0, 1.0 - p.flightTime / 1.2);
        double v0Score  = Math.max(0.0, 1.0 - p.v0 / 20.0);
        double eaScore  = Math.min(1.0, p.entryAngle / 65.0);
        double rpmScore = Math.max(0.0, 1.0 - motorRPM / 6000.0);
        double[] w      = ShooterConstants.SHOT_SCORE_WEIGHTS;
        return w[0]*tScore + w[1]*v0Score + w[2]*eaScore + w[3]*rpmScore;
    }

    /**
     * Convert exit speed v₀ to motor RPM — exact formula from the HTML RPM tab:
     * <pre>
     *   surface_speed = v₀ / η
     *   wheel_RPM     = (surface_speed / (π × diameter)) × 60
     *   motor_RPM     = wheel_RPM × gear_ratio
     * </pre>
     */
    private static double toMotorRPM(double v0) {
        double surfaceSpeed = v0 / ETA;
        double wheelRPM     = (surfaceSpeed / (Math.PI * WHEEL_DIAM_M)) * 60.0;
        return wheelRPM * GEAR_RATIO;
    }

    /** Internal physics result — not exposed publicly. */
    private record Physics(double v0, double flightTime, double entryAngle) {}
}

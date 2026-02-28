package frc.robot.util;

import frc.robot.Constants.ShooterConstants;

/**
 * Stateless ballistics calculator for FRC 2026.
 *
 * File location: src/main/java/frc/robot/util/ShotCalculator.java
 *
 * Implements the exact physics model from frc_v0_calculator.html:
 *
 *   v₀ = d · √( g / (2·cos²θ · (d·tanθ − Δh)) )
 *   t  = d / (v₀·cosθ)
 *   vy_final = v₀·sinθ − g·t
 *   entry_angle = atan2(−vy_final, v₀·cosθ)   [degrees, positive = downward]
 *
 *   surfaceSpeed = v₀ / efficiency
 *   motorRPM     = (surfaceSpeed / (π·wheelDiam)) · 60 · gearRatio
 *
 * All methods are static — this class has no state and is thread-safe.
 */
public final class ShotCalculator {

    // Pre-computed constant — avoids multiplying 0.0254 every loop iteration
    private static final double WHEEL_DIAM_METERS =
        ShooterConstants.WHEEL_DIAMETER_INCHES * 0.0254;

    // ── ShotResult ────────────────────────────────────────────────────────────

    /** Immutable result of one shot calculation. All fields are public final. */
    public static final class ShotResult {
        /** Required launch speed at shooter exit (m/s). */
        public final double launchSpeedMps;
        /** Optimal launch angle above horizontal (degrees). */
        public final double launchAngleDeg;
        /** Time of flight (seconds). */
        public final double flightTimeSec;
        /** Angle at which the game piece enters the hub (degrees, positive = downward). */
        public final double entryAngleDeg;
        /** Required motor RPM. */
        public final double motorRPM;
        /** Horizontal shooter-to-hub distance used (meters). */
        public final double distanceMeters;
        /** Optimiser score in [0, 1]. Higher = better. */
        public final double score;

        private ShotResult(double launchSpeedMps, double launchAngleDeg,
                double flightTimeSec, double entryAngleDeg,
                double motorRPM, double distanceMeters, double score) {
            this.launchSpeedMps = launchSpeedMps;
            this.launchAngleDeg = launchAngleDeg;
            this.flightTimeSec  = flightTimeSec;
            this.entryAngleDeg  = entryAngleDeg;
            this.motorRPM       = motorRPM;
            this.distanceMeters = distanceMeters;
            this.score          = score;
        }

        /**
         * True when the result is physically valid and within hardware limits.
         * A false result means no shot should be attempted.
         */
        public boolean isValid() {
            return Double.isFinite(launchSpeedMps)
                && launchSpeedMps > 0.0
                && motorRPM       <= ShooterConstants.MAX_MOTOR_RPM
                && entryAngleDeg  >= ShooterConstants.MIN_ENTRY_ANGLE_DEG;
        }

        @Override
        public String toString() {
            return String.format(
                "ShotResult{θ=%.1f°, v₀=%.2f m/s, RPM=%.0f, entry=%.1f°, t=%.3f s, d=%.2f m, score=%.3f}",
                launchAngleDeg, launchSpeedMps, motorRPM,
                entryAngleDeg, flightTimeSec, distanceMeters, score);
        }
    }

    // ── Main API ──────────────────────────────────────────────────────────────

    /**
     * Find the highest-scoring valid shot for this distance by sweeping all
     * candidate angles in ANGLE_SWEEP_STEP_DEG increments.
     *
     * @param distanceMeters Horizontal distance from shooter exit to hub center (m).
     * @return Best ShotResult, or null if no valid angle exists at this distance.
     */
    public static ShotResult calculate(double distanceMeters) {
        if (distanceMeters <= 0.0) return null;

        ShotResult best = null;
        for (double deg = ShooterConstants.MIN_LAUNCH_ANGLE_DEG;
                deg <= ShooterConstants.MAX_LAUNCH_ANGLE_DEG;
                deg += ShooterConstants.ANGLE_SWEEP_STEP_DEG) {

            ShotResult candidate = computeAtAngle(distanceMeters, deg);
            if (candidate == null || !candidate.isValid()) continue;
            if (best == null || candidate.score > best.score) {
                best = candidate;
            }
        }
        return best;
    }

    /**
     * Compute ballistics at one specific angle.
     *
     * @param d        Horizontal distance (m).
     * @param angleDeg Launch angle (degrees above horizontal).
     * @return ShotResult, or null when the physics equation has no real solution.
     */
    public static ShotResult computeAtAngle(double d, double angleDeg) {
        if (d <= 0.0) return null;

        final double h1    = ShooterConstants.SHOOTER_EXIT_HEIGHT_METERS;
        final double h2    = ShooterConstants.HUB_TARGET_HEIGHT_METERS;
        final double deltaH = h2 - h1;

        final double th   = Math.toRadians(angleDeg);
        final double cosT = Math.cos(th);
        final double sinT = Math.sin(th);
        final double tanT = sinT / cosT;

        // Physics equation — denominator must be positive for a real solution
        final double denom = 2.0 * cosT * cosT * (d * tanT - deltaH);
        if (denom <= 0.0) return null;

        final double v0 = d * Math.sqrt(ShooterConstants.g / denom);
        if (!Double.isFinite(v0) || v0 <= 0.0) return null;

        final double t        = d / (v0 * cosT);
        final double vyFinal  = v0 * sinT - ShooterConstants.g * t;
        final double entryDeg = Math.toDegrees(Math.atan2(-vyFinal, v0 * cosT));

        final double surfaceSpeed = v0 / ShooterConstants.EFFICIENCY;
        final double wheelRPM     = (surfaceSpeed / (Math.PI * WHEEL_DIAM_METERS)) * 60.0;
        final double motorRPM     = wheelRPM * ShooterConstants.SHOOTER_GEAR_RATIO;

        return new ShotResult(v0, angleDeg, t, entryDeg, motorRPM, d,
                              score(t, v0, entryDeg, motorRPM));
    }

    // ── Scoring ───────────────────────────────────────────────────────────────

    /**
     * Score a shot on [0, 1].
     * Weights from ShooterConstants.SHOT_SCORE_WEIGHTS = [flightTime, v0, entryAngle, rpm].
     * Scoring from frc_v0_calculator.html (accuracy-biased by default).
     */
    private static double score(double t, double v0, double entryDeg, double motorRPM) {
        final double[] w = ShooterConstants.SHOT_SCORE_WEIGHTS;

        double tScore   = Math.max(0.0, 1.0 - t / 1.2);
        double v0Score  = Math.max(0.0, 1.0 - v0 / 20.0);
        double eaScore  = Math.min(1.0, entryDeg / 65.0); // steeper = more accurate
        double rpmScore = Math.max(0.0, 1.0 - motorRPM / ShooterConstants.MAX_MOTOR_RPM);

        return w[0] * tScore + w[1] * v0Score + w[2] * eaScore + w[3] * rpmScore;
    }

    private ShotCalculator() {}
}
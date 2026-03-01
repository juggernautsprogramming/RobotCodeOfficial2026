package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * VelocityCompensator — iterative vector-based "shoot-on-the-move" correction.
 *
 * <h3>Physics model</h3>
 * During time-of-flight {@code tof}, the robot moves by
 * {@code (vx·tof, vy·tof)} in field space.  To still hit the stationary hub we
 * aim at a <em>virtual hub</em> that is displaced by the negative of that motion:
 * <pre>
 *   virtualHub = realHub − robotVelocity × tof
 *   aimVector  = virtualHub − robotPose
 *   tof_new    = |aimVector| / V_BALL
 *   → repeat until |Δdist| &lt; 1 mm
 * </pre>
 * Three iterations converge to sub-millimetre error in all realistic cases.
 */
public final class VelocityCompensator {

    /** Approximate ball exit speed (m/s). Tune to match your flywheel at setpoint. */
    private static final double V_BALL = 15.0;

    /** Maximum Newton refinement passes. */
    private static final int MAX_ITERATIONS = 5;

    /** Stop refining when distance change drops below this (metres). */
    private static final double CONVERGENCE_M = 0.001;

    // ── Singleton ─────────────────────────────────────────────────────────────
    private static final VelocityCompensator INSTANCE = new VelocityCompensator();

    public static VelocityCompensator getInstance() { return INSTANCE; }

    private VelocityCompensator() {}

    // ── Public result record ──────────────────────────────────────────────────

    /**
     * @param virtualDistanceMeters Effective distance to the virtual hub — feed to {@link ShotCalculator}.
     * @param aimHeadingDeg         Field-relative heading the robot must face (degrees, 0=+X axis).
     * @param timeOfFlight          Estimated ball flight time (seconds).
     */
    public record CompensationResult(
            double virtualDistanceMeters,
            double aimHeadingDeg,
            double timeOfFlight) {}

    // ── Core API ──────────────────────────────────────────────────────────────

    /**
     * Compute the velocity-compensated aim heading and effective shot distance.
     *
     * @param robotPose  Robot centre, field-relative (metres).
     * @param hubPose    Hub centre, field-relative (metres).
     * @param robotVx    Robot velocity X, field-relative (m/s).
     * @param robotVy    Robot velocity Y, field-relative (m/s).
     */
    public CompensationResult compensate(
            Translation2d robotPose,
            Translation2d hubPose,
            double robotVx,
            double robotVy) {

        double dist = robotPose.getDistance(hubPose);
        double tof  = dist / V_BALL;

        for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
            Translation2d virtualHub =
                hubPose.minus(new Translation2d(robotVx * tof, robotVy * tof));

            Translation2d aimVec  = virtualHub.minus(robotPose);
            double        newDist = aimVec.getNorm();
            double        newTof  = newDist / V_BALL;

            if (Math.abs(newDist - dist) < CONVERGENCE_M) {
                double heading = Math.toDegrees(Math.atan2(aimVec.getY(), aimVec.getX()));
                return new CompensationResult(newDist, heading, newTof);
            }
            dist = newDist;
            tof  = newTof;
        }

        // Best estimate after max iterations
        Translation2d virtualHub = hubPose.minus(new Translation2d(robotVx * tof, robotVy * tof));
        Translation2d aimVec     = virtualHub.minus(robotPose);
        double        heading    = Math.toDegrees(Math.atan2(aimVec.getY(), aimVec.getX()));
        return new CompensationResult(dist, heading, tof);
    }
}
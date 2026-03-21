/*
 * FireControlSolver.java - Newton-method SOTM fire control with drag compensation
 *
 * Adapted from frc-fire-control (MIT License, Copyright (c) 2026 FRC Team 5962 perSEVERE)
 * https://github.com/eeveemara/frc-fire-control
 *
 * Replaces VelocityCompensator + ShotCalculator for shoot-on-the-move scenarios.
 * Key improvements over the old system:
 *   - Newton's method for self-consistent time-of-flight (vs. fixed-point iteration)
 *   - Drag compensation: inherited velocity decays in flight (1 - e^(-c*tof)) / c
 *   - Second-order latency prediction: accounts for robot acceleration, not just velocity
 *   - Confidence scoring: 0-100 weighted geometric mean — use as additional fire gate
 *   - Angular velocity feedforward: reduces heading lag during rotation
 */
package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Shoot-on-the-move fire control solver. Computes RPM and aim heading while driving.
 *
 * <p>Usage: instantiate in ShooterSubsystem, populate the LUT via {@link #loadLUTEntry},
 * then call {@link #calculate} once per loop cycle from DriveToHubAndShootCommand.
 *
 * <pre>
 *   FireControlSolver.ShotInputs inputs = new FireControlSolver.ShotInputs(
 *       drivetrain.getPose(), drivetrain.getFieldSpeeds(), drivetrain.getRobotSpeeds(),
 *       HUB_CENTER, new Translation2d(0, 0), 1.0);
 *   FireControlSolver.LaunchParameters result = solver.calculate(inputs);
 *   if (result.isValid() &amp;&amp; result.confidence() &gt; 40) {
 *       shooter.setRPM(result.rpm());
 *       drive.setHeading(result.driveAngle());
 *   }
 * </pre>
 */
public class FireControlSolver {

    /** The result of calculate(). RPM, time-of-flight, aim heading, and a 0-100 confidence score. */
    public record LaunchParameters(
            double rpm,
            double timeOfFlightSec,
            Rotation2d driveAngle,
            double driveAngularVelocityRadPerSec,
            boolean isValid,
            double confidence,
            double solvedDistanceM,
            int iterationsUsed,
            boolean warmStartUsed) {

        public static final LaunchParameters INVALID =
                new LaunchParameters(0, 0, new Rotation2d(), 0, false, 0, 0, 0, false);
    }

    /**
     * All the state the solver needs from your robot each cycle.
     * pitchDeg/rollDeg are chassis tilt angles — pass 0 if not available and set
     * config.maxTiltDeg = 90 to disable tilt suppression.
     */
    public record ShotInputs(
            Pose2d robotPose,
            ChassisSpeeds fieldVelocity,
            ChassisSpeeds robotVelocity,
            Translation2d hubCenter,
            Translation2d hubForward,
            double visionConfidence,
            double pitchDeg,
            double rollDeg) {

        /** Convenience constructor when pitch/roll data is unavailable. */
        public ShotInputs(
                Pose2d robotPose,
                ChassisSpeeds fieldVelocity,
                ChassisSpeeds robotVelocity,
                Translation2d hubCenter,
                Translation2d hubForward,
                double visionConfidence) {
            this(robotPose, fieldVelocity, robotVelocity, hubCenter, hubForward, visionConfidence, 0.0, 0.0);
        }
    }

    /** Tuning parameters. Wire to SmartDashboard or TunableNumber for live adjustment. */
    public static class Config {
        // Launcher geometry — measure from CAD
        public double launcherOffsetX = 0.20; // meters forward of robot center
        public double launcherOffsetY = 0.0;  // meters left of robot center

        // Scoring range (meters)
        public double minScoringDistance = 0.5;
        public double maxScoringDistance = 5.0;

        // Newton solver tuning
        public int maxIterations = 25;
        public double convergenceTolerance = 0.001; // seconds
        public double tofMin = 0.05;
        public double tofMax = 5.0;

        // Below this robot speed (m/s), skip SOTM and aim straight
        public double minSOTMSpeed = 0.1;
        // Above this speed (m/s), return INVALID (outside calibration range)
        public double maxSOTMSpeed = 3.0;

        // Latency compensation (ms)
        public double phaseDelayMs = 30.0;  // vision pipeline lag
        public double mechLatencyMs = 20.0; // mechanism response time

        // Drag coefficient on inherited velocity. Real displacement = (1-e^(-c*tof))/c.
        // Set to 0 to disable drag compensation.
        public double sotmDragCoeff = 0.47;

        // Confidence scoring weights (5-component weighted geometric mean)
        public double wConvergence = 1.0;
        public double wVelocityStability = 0.8;
        public double wVisionConfidence = 1.2;
        public double wHeadingAccuracy = 1.5;
        public double wDistanceInRange = 0.5;
        public double headingMaxErrorRad = Math.toRadians(15);

        // Tolerance tightens with robot speed: scaledMaxError = base / (1 + scalar * speed)
        public double headingSpeedScalar = 1.0;
        // Tolerance tightens at longer range: scaledMaxError *= refDist / distance
        public double headingReferenceDistance = 2.5; // meters

        // Suppress firing when chassis is tilted beyond this angle (degrees). Set to 90 to disable.
        public double maxTiltDeg = 5.0;
    }

    private final Config config;

    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap correctionRpmMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap correctionTofMap = new InterpolatingDoubleTreeMap();

    private double rpmOffset = 0;

    // Warm-start state (reused across cycles to avoid reallocation)
    private double previousTOF = -1;
    private double previousSpeed = 0;
    private double prevRobotVx = 0;
    private double prevRobotVy = 0;
    private double prevRobotOmega = 0;

    public FireControlSolver(Config config) {
        this.config = config;
    }

    public FireControlSolver() {
        this(new Config());
    }

    /** Add a distance/RPM/TOF entry to the lookup table. Use FireControlSimulator to generate these. */
    public void loadLUTEntry(double distanceM, double rpm, double tof) {
        rpmMap.put(distanceM, rpm);
        tofMap.put(distanceM, tof);
    }

    double effectiveRPM(double distance) {
        double base = rpmMap.get(distance);
        Double correction = correctionRpmMap.get(distance);
        return base + (correction != null ? correction : 0.0) + rpmOffset;
    }

    double effectiveTOF(double distance) {
        double base = tofMap.get(distance);
        Double correction = correctionTofMap.get(distance);
        return base + (correction != null ? correction : 0.0);
    }

    private double dragCompensatedTOF(double tof) {
        double c = config.sotmDragCoeff;
        if (c < 1e-6) return tof;
        return (1.0 - Math.exp(-c * tof)) / c;
    }

    private static final double DERIV_H = 0.01; // 1 cm step for derivative

    double tofMapDerivative(double d) {
        double tHigh = effectiveTOF(d + DERIV_H);
        double tLow  = effectiveTOF(d - DERIV_H);
        return (tHigh - tLow) / (2.0 * DERIV_H);
    }

    /**
     * Solve for the firing solution. Call once per 20 ms cycle in robotPeriodic() or execute().
     * Returns {@link LaunchParameters#INVALID} if out of range, behind hub, tilted, or going too fast.
     */
    public LaunchParameters calculate(ShotInputs inputs) {
        if (inputs == null || inputs.robotPose() == null
                || inputs.fieldVelocity() == null || inputs.robotVelocity() == null) {
            return LaunchParameters.INVALID;
        }

        Pose2d rawPose = inputs.robotPose();
        double poseX = rawPose.getX();
        double poseY = rawPose.getY();
        if (Double.isNaN(poseX) || Double.isNaN(poseY)
                || Double.isInfinite(poseX) || Double.isInfinite(poseY)) {
            return LaunchParameters.INVALID;
        }

        // Second-order latency prediction: v*dt + 0.5*a*dt^2
        // Better than first-order through turns and speed changes.
        ChassisSpeeds robotVel = inputs.robotVelocity();
        double dt = config.phaseDelayMs / 1000.0;
        double ax = (robotVel.vxMetersPerSecond - prevRobotVx) / 0.02;
        double ay = (robotVel.vyMetersPerSecond - prevRobotVy) / 0.02;
        double aOmega = (robotVel.omegaRadiansPerSecond - prevRobotOmega) / 0.02;
        Pose2d compensatedPose = rawPose.exp(new Twist2d(
                robotVel.vxMetersPerSecond * dt + 0.5 * ax * dt * dt,
                robotVel.vyMetersPerSecond * dt + 0.5 * ay * dt * dt,
                robotVel.omegaRadiansPerSecond * dt + 0.5 * aOmega * dt * dt));
        prevRobotVx    = robotVel.vxMetersPerSecond;
        prevRobotVy    = robotVel.vyMetersPerSecond;
        prevRobotOmega = robotVel.omegaRadiansPerSecond;

        double robotX  = compensatedPose.getX();
        double robotY  = compensatedPose.getY();
        double heading = compensatedPose.getRotation().getRadians();

        Translation2d hubCenter  = inputs.hubCenter();
        double hubX = hubCenter.getX();
        double hubY = hubCenter.getY();

        // Behind-hub detection: dot product with hub forward vector.
        // Pass Translation2d(0,0) for hubForward to bypass this check.
        Translation2d hubForward = inputs.hubForward();
        if (hubForward.getNorm() > 1e-6) {
            double dot = (hubX - robotX) * hubForward.getX() + (hubY - robotY) * hubForward.getY();
            if (dot < 0) return LaunchParameters.INVALID;
        }

        // Tilt gate — suppress firing while robot is on a bump or ramp
        if (Math.abs(inputs.pitchDeg()) > config.maxTiltDeg
                || Math.abs(inputs.rollDeg()) > config.maxTiltDeg) {
            return LaunchParameters.INVALID;
        }

        // Transform robot center to launcher position
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);
        double launcherX = robotX + config.launcherOffsetX * cosH - config.launcherOffsetY * sinH;
        double launcherY = robotY + config.launcherOffsetX * sinH + config.launcherOffsetY * cosH;

        // Launcher velocity includes rotational component: v_launcher = v_robot + omega x r
        ChassisSpeeds fieldVel = inputs.fieldVelocity();
        double launcherFieldOffX = config.launcherOffsetX * cosH - config.launcherOffsetY * sinH;
        double launcherFieldOffY = config.launcherOffsetX * sinH + config.launcherOffsetY * cosH;
        double omega = fieldVel.omegaRadiansPerSecond;
        double vx = fieldVel.vxMetersPerSecond + (-launcherFieldOffY) * omega;
        double vy = fieldVel.vyMetersPerSecond + launcherFieldOffX * omega;

        // Displacement from launcher to hub
        double rx = hubX - launcherX;
        double ry = hubY - launcherY;
        double distance = Math.hypot(rx, ry);

        if (distance < config.minScoringDistance || distance > config.maxScoringDistance) {
            return LaunchParameters.INVALID;
        }

        double robotSpeed = Math.hypot(vx, vy);
        if (robotSpeed > config.maxSOTMSpeed) return LaunchParameters.INVALID;

        boolean velocityFiltered = robotSpeed < config.minSOTMSpeed;

        double solvedTOF;
        double projDist;
        int iterationsUsed;
        boolean warmStartUsed;

        if (velocityFiltered) {
            solvedTOF     = effectiveTOF(distance);
            projDist      = distance;
            iterationsUsed = 0;
            warmStartUsed = false;
        } else {
            // Newton-method SOTM solver
            int    maxIter  = config.maxIterations;
            double convTol  = config.convergenceTolerance;

            double tof;
            if (previousTOF > 0) {
                tof           = previousTOF;
                warmStartUsed = true;
            } else {
                tof           = effectiveTOF(distance);
                warmStartUsed = false;
            }

            projDist       = distance;
            iterationsUsed = 0;

            for (int i = 0; i < maxIter; i++) {
                double prevTOF = tof;

                double c       = config.sotmDragCoeff;
                double dragExp = c < 1e-6 ? 1.0 : Math.exp(-c * tof);
                double driftTOF = c < 1e-6 ? tof : (1.0 - dragExp) / c;

                double prx = rx - vx * driftTOF;
                double pry = ry - vy * driftTOF;
                projDist = Math.hypot(prx, pry);

                if (projDist < 0.01) {
                    tof = effectiveTOF(distance);
                    iterationsUsed = maxIter + 1;
                    break;
                }

                double lookupTOF = effectiveTOF(projDist);
                double dPrime    = -dragExp * (prx * vx + pry * vy) / projDist;
                double gPrime    = tofMapDerivative(projDist);
                double f         = lookupTOF - tof;
                double fPrime    = gPrime * dPrime - 1.0;

                if (Math.abs(fPrime) > 0.01) {
                    tof = tof - f / fPrime;
                } else {
                    tof = lookupTOF; // fixed-point fallback
                }

                tof = MathUtil.clamp(tof, config.tofMin, config.tofMax);
                iterationsUsed = i + 1;

                if (Math.abs(tof - prevTOF) < convTol) break;
            }

            if (tof > config.tofMax || tof < 0.0 || Double.isNaN(tof)) {
                tof = effectiveTOF(distance);
                iterationsUsed = maxIter + 1;
            }

            solvedTOF = tof;
        }

        previousTOF = solvedTOF;

        double effectiveTOFVal = solvedTOF + config.mechLatencyMs / 1000.0;
        double effectiveRPMVal = effectiveRPM(projDist);

        // Aim at velocity-compensated target position
        double compTargetX, compTargetY;
        if (velocityFiltered) {
            compTargetX = hubX;
            compTargetY = hubY;
        } else {
            double headingDriftTOF = dragCompensatedTOF(solvedTOF);
            compTargetX = hubX - vx * headingDriftTOF;
            compTargetY = hubY - vy * headingDriftTOF;
        }
        double aimX = compTargetX - robotX;
        double aimY = compTargetY - robotY;
        Rotation2d driveAngle = new Rotation2d(aimX, aimY);

        double headingErrorRad = MathUtil.angleModulus(driveAngle.getRadians() - heading);

        // Angular velocity feedforward: tangential_vel / distance²
        double driveAngularVelocity = 0;
        if (!velocityFiltered && distance > 0.1) {
            double tangentialVel = (ry * vx - rx * vy) / distance;
            driveAngularVelocity = tangentialVel / distance;
        }

        // Solver convergence quality for confidence scoring
        double solverQuality;
        if (velocityFiltered) {
            solverQuality = 1.0;
        } else {
            int maxIter = config.maxIterations;
            if (iterationsUsed > maxIter) {
                solverQuality = 0.0;
            } else if (iterationsUsed <= 3) {
                solverQuality = 1.0;
            } else {
                solverQuality = MathUtil.interpolate(1.0, 0.1, (double) (iterationsUsed - 3) / (maxIter - 3));
            }
        }

        double confidence = computeConfidence(solverQuality, robotSpeed, headingErrorRad, distance, inputs.visionConfidence());
        previousSpeed = robotSpeed;

        return new LaunchParameters(
                effectiveRPMVal,
                effectiveTOFVal,
                driveAngle,
                driveAngularVelocity,
                true,
                confidence,
                distance,
                iterationsUsed,
                warmStartUsed);
    }

    /**
     * Confidence score 0-100. Weighted geometric mean of 5 factors.
     * If any single factor drops to zero, the whole score goes to zero — intentional.
     */
    private double computeConfidence(
            double solverQuality, double currentSpeed, double headingErrorRad,
            double distance, double visionConfidence) {

        double convergenceQuality = solverQuality;

        double speedDelta       = Math.abs(currentSpeed - previousSpeed);
        double velocityStability = MathUtil.clamp(1.0 - speedDelta / 0.5, 0, 1);

        double visionConf = MathUtil.clamp(visionConfidence, 0, 1);

        double distanceScale  = MathUtil.clamp(config.headingReferenceDistance / distance, 0.5, 2.0);
        double speedScale     = 1.0 / (1.0 + config.headingSpeedScalar * currentSpeed);
        double scaledMaxError = config.headingMaxErrorRad * distanceScale * speedScale;
        double headingAccuracy = MathUtil.clamp(1.0 - Math.abs(headingErrorRad) / scaledMaxError, 0, 1);

        double rangeSpan     = config.maxScoringDistance - config.minScoringDistance;
        double rangeFraction = (distance - config.minScoringDistance) / rangeSpan;
        double distInRange   = MathUtil.clamp(1.0 - 2.0 * Math.abs(rangeFraction - 0.5), 0, 1);

        double[] c = {convergenceQuality, velocityStability, visionConf, headingAccuracy, distInRange};
        double[] w = {config.wConvergence, config.wVelocityStability, config.wVisionConfidence,
                      config.wHeadingAccuracy, config.wDistanceInRange};

        double sumW = 0, logSum = 0;
        for (int i = 0; i < 5; i++) {
            if (c[i] <= 0) return 0;
            logSum += w[i] * Math.log(c[i]);
            sumW   += w[i];
        }

        if (sumW <= 0) return 0;
        return MathUtil.clamp(Math.exp(logSum / sumW) * 100.0, 0, 100);
    }

    /** Add a per-distance RPM correction on top of the base LUT. Good for field tuning at competition. */
    public void addRpmCorrection(double distance, double deltaRpm) {
        correctionRpmMap.put(distance, deltaRpm);
    }

    /** Clear all corrections, back to the raw LUT. */
    public void clearCorrections() {
        correctionRpmMap.clear();
        correctionTofMap.clear();
    }

    /** Bump the RPM offset by delta. Clamped to +/- 200. Bind to copilot D-pad for match tuning. */
    public void adjustOffset(double delta) {
        rpmOffset = MathUtil.clamp(rpmOffset + delta, -200, 200);
    }

    /** Reset the RPM offset to zero. Call this on mode transitions. */
    public void resetOffset() {
        rpmOffset = 0;
    }

    public double getOffset() { return rpmOffset; }

    /** Raw time-of-flight from the LUT at this distance (no velocity compensation). */
    public double getTimeOfFlight(double distanceM) { return effectiveTOF(distanceM); }

    /** Reset warm-start state. Call after a pose reset so the solver doesn't use stale data. */
    public void resetWarmStart() {
        previousTOF    = -1;
        previousSpeed  = 0;
        prevRobotVx    = 0;
        prevRobotVy    = 0;
        prevRobotOmega = 0;
    }
}

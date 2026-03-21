/*
 * FireControlSimulator.java - RK4 projectile physics with drag and Magnus lift
 *
 * Adapted from frc-fire-control (MIT License, Copyright (c) 2026 FRC Team 5962 perSEVERE)
 * https://github.com/eeveemara/frc-fire-control
 *
 * Used at robot startup to generate the distance→RPM/TOF lookup table that
 * FireControlSolver needs. Run generateLUT() once in ShooterSubsystem constructor.
 * Takes ~200ms. Uses physics-derived TOF values (which are hard to measure empirically),
 * and these are blended with the empirical RPM table for best accuracy.
 */
package frc.robot.subsystems.Shooter;

import java.util.ArrayList;
import java.util.List;

/**
 * Simulates ball flight with drag and Magnus lift using RK4 integration.
 * Binary-searches RPM to hit the target height at each distance.
 * Generates a 91-point LUT from 0.50m to 5.00m.
 *
 * <pre>
 *   FireControlSimulator.SimParameters params = new FireControlSimulator.SimParameters(
 *       0.215, 0.1501, 0.47, 0.0, 1.225,  // ball, drag, Magnus, air density
 *       0.52, 0.1016, 2.64,                // exit height, wheel diameter, target height
 *       0.424, 61.5,                        // slip factor, launch angle
 *       0.001, 1500, 6000, 25, 5.0);       // sim params
 *   FireControlSimulator sim = new FireControlSimulator(params);
 *   FireControlSimulator.GeneratedLUT lut = sim.generateLUT();
 * </pre>
 */
public class FireControlSimulator {

    /** Physical parameters for your robot. Measure from CAD and the game manual. */
    public record SimParameters(
            double ballMassKg,
            double ballDiameterM,
            double dragCoeff,
            double magnusCoeff,
            double airDensity,
            double exitHeightM,
            double wheelDiameterM,
            double targetHeightM,
            double slipFactor,
            double fixedLaunchAngleDeg,
            double dt,
            double rpmMin,
            double rpmMax,
            int binarySearchIters,
            double maxSimTime) {}

    public record TrajectoryResult(
            double zAtTarget, double tof, boolean reachedTarget, double maxHeight, double apexX) {}

    /** One row of the lookup table: distance -> RPM + TOF. */
    public record LUTEntry(double distanceM, double rpm, double tof, boolean reachable) {}

    /** Full LUT with generation statistics. */
    public record GeneratedLUT(
            List<LUTEntry> entries,
            SimParameters params,
            int reachableCount,
            int unreachableCount,
            double maxRangeM,
            long generationTimeMs) {}

    private final SimParameters params;

    // Precomputed aerodynamic constants
    private final double kDrag;
    private final double kMagnus;

    public FireControlSimulator(SimParameters params) {
        this.params = params;
        double area = Math.PI * (params.ballDiameterM() / 2.0) * (params.ballDiameterM() / 2.0);
        this.kDrag   = (params.airDensity() * params.dragCoeff()   * area) / (2.0 * params.ballMassKg());
        this.kMagnus = (params.airDensity() * params.magnusCoeff() * area) / (2.0 * params.ballMassKg());
    }

    /** Convert motor RPM to ball exit speed (m/s), accounting for slip. */
    public double exitVelocity(double rpm) {
        return params.slipFactor() * rpm * Math.PI * params.wheelDiameterM() / 60.0;
    }

    /** Simulate a ball launched at the given RPM and find its height when it reaches targetDistanceM. */
    public TrajectoryResult simulate(double rpm, double targetDistanceM) {
        double v0       = exitVelocity(rpm);
        double launchRad = Math.toRadians(params.fixedLaunchAngleDeg());
        double vx       = v0 * Math.cos(launchRad);
        double vz       = v0 * Math.sin(launchRad);

        double x        = 0;
        double z        = params.exitHeightM();
        double dt       = params.dt();
        double maxHeight = z;
        double apexX    = 0;
        double t        = 0;

        while (t < params.maxSimTime()) {
            double[] state = {x, z, vx, vz};
            double[] k1    = derivatives(state);
            double[] s2    = addScaled(state, k1, dt / 2.0);
            double[] k2    = derivatives(s2);
            double[] s3    = addScaled(state, k2, dt / 2.0);
            double[] k3    = derivatives(s3);
            double[] s4    = addScaled(state, k3, dt);
            double[] k4    = derivatives(s4);

            x  += dt / 6.0 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
            z  += dt / 6.0 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
            vx += dt / 6.0 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]);
            vz += dt / 6.0 * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]);
            t  += dt;

            if (z > maxHeight) { maxHeight = z; apexX = x; }

            if (x >= targetDistanceM) {
                double prevX  = x - vx * dt;
                double prevZ  = z - vz * dt;
                double frac   = (targetDistanceM - prevX) / (x - prevX);
                double zAt    = prevZ + frac * (z - prevZ);
                double tofAt  = t - dt + frac * dt;
                return new TrajectoryResult(zAt, tofAt, true, maxHeight, apexX);
            }

            if (z < 0) return new TrajectoryResult(0, t, false, maxHeight, apexX);
        }

        return new TrajectoryResult(0, params.maxSimTime(), false, maxHeight, apexX);
    }

    // state = [x, z, vx, vz]
    // ax = -kDrag * |v| * vx
    // az = -g - kDrag * |v| * vz + kMagnus * |v|^2 (Magnus = backspin upward lift)
    private double[] derivatives(double[] state) {
        double svx   = state[2];
        double svz   = state[3];
        double speed = Math.hypot(svx, svz);

        double ax = -kDrag * speed * svx;
        double az = -9.81 - kDrag * speed * svz + kMagnus * speed * speed;

        return new double[]{svx, svz, ax, az};
    }

    private static double[] addScaled(double[] base, double[] delta, double scale) {
        return new double[]{
            base[0] + delta[0] * scale,
            base[1] + delta[1] * scale,
            base[2] + delta[2] * scale,
            base[3] + delta[3] * scale
        };
    }

    /** Binary search for the RPM that puts the ball at the target height at the given distance. */
    public LUTEntry findRPMForDistance(double distanceM) {
        double heightTol = 0.02; // 2 cm
        double lo = params.rpmMin();
        double hi = params.rpmMax();

        TrajectoryResult maxCheck = simulate(hi, distanceM);
        if (!maxCheck.reachedTarget()) {
            return new LUTEntry(distanceM, 0, 0, false);
        }

        double bestRpm   = hi;
        double bestTof   = maxCheck.tof();
        double bestError = Math.abs(maxCheck.zAtTarget() - params.targetHeightM());

        for (int i = 0; i < params.binarySearchIters(); i++) {
            double mid    = (lo + hi) / 2.0;
            TrajectoryResult result = simulate(mid, distanceM);

            if (!result.reachedTarget()) { lo = mid; continue; }

            double error    = result.zAtTarget() - params.targetHeightM();
            double absError = Math.abs(error);

            if (absError < bestError) {
                bestRpm   = mid;
                bestTof   = result.tof();
                bestError = absError;
            }

            if (absError < heightTol) return new LUTEntry(distanceM, mid, result.tof(), true);

            if (error > 0) hi = mid; else lo = mid;
        }

        return new LUTEntry(distanceM, bestRpm, bestTof, bestError < 0.10);
    }

    /**
     * Generate the full lookup table: 0.50m to 5.00m in 5 cm steps (91 entries).
     * Takes approximately 200ms. Call once at robot startup from ShooterSubsystem constructor.
     */
    public GeneratedLUT generateLUT() {
        long startMs = System.currentTimeMillis();
        List<LUTEntry> entries    = new ArrayList<>();
        int reachable   = 0;
        int unreachable = 0;
        double maxRange = 0;

        for (int i = 0; i <= 90; i++) {
            double distance = Math.round((0.50 + i * 0.05) * 100.0) / 100.0;
            LUTEntry entry  = findRPMForDistance(distance);
            entries.add(entry);

            if (entry.reachable()) {
                reachable++;
                maxRange = distance;
            } else {
                unreachable++;
            }
        }

        long elapsed = System.currentTimeMillis() - startMs;
        return new GeneratedLUT(entries, params, reachable, unreachable, maxRange, elapsed);
    }
}

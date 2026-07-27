package org.firstinspires.ftc.teamcode.pedroPathing.identification;

import java.util.ArrayList;
import java.util.List;

/**
 * Fits the first-order plant {@code u ≈ kS·sign(v) + kV·v + kA·a} used by Foresight.
 * Steady-state multi-power samples identify {@code kS} and {@code kV}; a constant-power
 * step response identifies {@code K}, {@code tau}, and plant {@code kA = tau / K}.
 */
final class PlantIdentificationMath {
    private PlantIdentificationMath() {}

    static final class SteadyFit {
        final double kS;
        final double kV;

        SteadyFit(double kS, double kV) {
            this.kS = kS;
            this.kV = kV;
        }
    }

    static final class StepFit {
        final double K;
        final double tau;
        /** Unscaled plant velocity gain {@code 1/K}. */
        final double kV;
        /** Unscaled plant acceleration gain {@code tau/K}. */
        final double kA;

        StepFit(double K, double tau) {
            this.K = K;
            this.tau = tau;
            this.kV = 1.0 / K;
            this.kA = tau / K;
        }
    }

    /**
     * Linear fit of commanded power magnitude vs steady speed: {@code |u| = kS + kV·|v|}.
     * Points are {@code (speed, power)} with both non-negative.
     */
    static SteadyFit fitSteadyState(List<double[]> speedPowerPoints) {
        if (speedPowerPoints.size() < 2) {
            throw new IllegalArgumentException("Need at least two steady-state samples.");
        }
        double[] x = new double[speedPowerPoints.size()];
        double[] y = new double[speedPowerPoints.size()];
        for (int i = 0; i < speedPowerPoints.size(); i++) {
            x[i] = speedPowerPoints.get(i)[0];
            y[i] = speedPowerPoints.get(i)[1];
        }
        double[] ab = linearFit(x, y);
        // y = a + b*x  =>  power = kS + kV * speed
        double kS = Math.max(0.0, ab[0]);
        double kV = Math.max(0.0, ab[1]);
        return new SteadyFit(kS, kV);
    }

    /**
     * First-order step identification: steady gain {@code K = v_ss / power}, time constant
     * from {@code ln(K - v/power)} vs time on the rising edge.
     */
    static StepFit fitStepResponse(List<Double> times, List<Double> speeds, double power, int steadySamples) {
        int n = times.size();
        if (n < 4 || speeds.size() != n) {
            throw new IllegalArgumentException("Failed step calibration: insufficient samples.");
        }
        if (power <= 0) {
            throw new IllegalArgumentException("Step power must be positive.");
        }

        int start = Math.max(0, n - steadySamples);
        double sum = 0;
        for (int i = start; i < n; i++) {
            sum += speeds.get(i);
        }
        double vSs = sum / (n - start);
        double K = vSs / power;
        if (K <= 1e-6) {
            throw new IllegalArgumentException("Failed step calibration: near-zero steady speed.");
        }

        List<Double> x = new ArrayList<>();
        List<Double> y = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            double normalized = speeds.get(i) / power;
            if (normalized > 0.8 * K || normalized < 0.1 * K) continue;
            double residual = K - normalized;
            if (residual <= 1e-9) continue;
            y.add(Math.log(residual));
            x.add(times.get(i));
        }
        if (x.size() < 3) {
            throw new IllegalArgumentException("Failed step calibration: not enough rising-edge samples.");
        }
        double[] linReg = linearFit(
                x.stream().mapToDouble(Double::doubleValue).toArray(),
                y.stream().mapToDouble(Double::doubleValue).toArray());
        if (Math.abs(linReg[1]) < 1e-9) {
            throw new IllegalArgumentException("Failed step calibration: flat log-regression.");
        }
        double tau = -1.0 / linReg[1];
        if (tau <= 0) {
            throw new IllegalArgumentException("Failed step calibration: non-positive tau.");
        }
        return new StepFit(K, tau);
    }

    /** Ordinary least squares for {@code y = a + b x}. Returns {@code {a, b}}. */
    static double[] linearFit(double[] x, double[] y) {
        int n = x.length;
        if (n != y.length || n < 2) {
            throw new IllegalArgumentException("linearFit needs matching arrays of length >= 2");
        }
        double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
        for (int i = 0; i < n; i++) {
            sumX += x[i];
            sumY += y[i];
            sumXY += x[i] * y[i];
            sumX2 += x[i] * x[i];
        }
        double denom = n * sumX2 - sumX * sumX;
        if (Math.abs(denom) < 1e-12) {
            throw new IllegalArgumentException("linearFit: singular design (constant x).");
        }
        double b = (n * sumXY - sumX * sumY) / denom;
        double a = (sumY - b * sumX) / n;
        return new double[] {a, b};
    }
}

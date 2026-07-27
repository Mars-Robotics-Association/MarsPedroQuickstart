package org.firstinspires.ftc.teamcode.pedroPathing.control;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.foresightConfig;

import android.annotation.SuppressLint;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Designs lateral translational feedback after plant gains are set.
 * Reports unscaled plant {@code kV}/{@code kA} for reference; use Strafe Plant
 * Identification for the {@code kS_y/kV_y/kA_y} values written into Constants.
 * Suggested {@code kP} goes on {@code lateralTranslationalController} (DoM D uses body vy).
 */
@TeleOp(name = "Strafe Translational Auto Tuner", group = "3")
public class StrafeTranslationalAutoTuner extends OpMode {
    public static double BETA_LARGE = 0.6;
    public static double BETA_SMALL = 0.9;

    private static final double POWER = 0.4;
    private static final double RUNTIME = 1.2;
    private static final int SAMPLES = 15;

    private double tau;
    private double K;
    private double plantKv;
    private double plantKa;
    private double vMax = 0;
    private final List<Double> times = new ArrayList<>();
    private final List<Double> velocities = new ArrayList<>();
    private final Timer timer = new Timer();
    private boolean done = false;
    private double lastTime = 0.0;
    private Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(Pose.zero());
        follower.update();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Group 3: lateral translational feedback (run after group 2 plant ID).");
        telemetry.addLine("Runs " + RUNTIME + " s at power " + POWER + ". Leave room.");
        telemetry.addLine("Put kP on foresightConfig.lateralTranslationalController.");
        telemetry.addLine("DoM D uses measured body-frame vy when kD is non-zero.");
        telemetry.update();
        follower.update();
    }

    @Override
    public void start() {
        timer.reset();
        lastTime = timer.seconds();
        follower.manual(0, POWER, 0);
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        double now = timer.seconds();
        double dt = now - lastTime;
        if (dt <= 0) dt = 1e-6;
        lastTime = now;

        follower.update();

        telemetry.addData("done", done);
        telemetry.addData("dt", String.format(Locale.US, "%.6f s", dt));

        if (!done) {
            times.add(timer.seconds());
            double lateralVelocity = Math.abs(follower.velocity().toVector2D()
                    .dot(Vector2D.polar(1, follower.pose().heading() + Math.PI / 2)));
            vMax = Math.max(vMax, lateralVelocity / POWER);
            velocities.add(lateralVelocity);
            telemetry.addData("velocity (in/s)", String.format(Locale.US, "%.4f", lateralVelocity));

            if (timer.seconds() >= RUNTIME) {
                done = true;
                systemIdentification();
                follower.manual(0, 0, 0);
            } else {
                follower.manual(0, POWER, 0);
                telemetry.update();
                return;
            }
        }

        double kPLarge = calculateKp(BETA_LARGE);
        double kPSmall = calculateKp(BETA_SMALL);

        telemetry.addLine("--- Plant reference (prefer dedicated plant ID for kS) ---");
        telemetry.addData("plant kV_y (=1/K, unscaled)", String.format(Locale.US, "%.5f", plantKv));
        telemetry.addData("plant kA_y (=tau/K, unscaled)", String.format(Locale.US, "%.5f", plantKa));
        telemetry.addLine("--- Feedback for lateralTranslationalController ---");
        telemetry.addData("kP (beta=" + BETA_LARGE + ")", String.format(Locale.US, "%.4f", kPLarge));
        telemetry.addData("kP (beta=" + BETA_SMALL + ")", String.format(Locale.US, "%.4f", kPSmall));
        telemetry.addData("Est tau (s)", String.format(Locale.US, "%.4f", tau));
        telemetry.addData("Est K (in/s per power)", String.format(Locale.US, "%.4f", K));
        telemetry.update();
    }

    private double calculateKp(double beta) {
        double detunedKa = plantKa * beta;
        double denominator = foresightConfig.linearBrakeCoefficients.get().get(1, 1)
                + 2.0 * foresightConfig.quadraticBrakeCoefficients.get().get(1, 1) * vMax;
        double discriminant = detunedKa - plantKv * denominator;

        if (discriminant < 0) return plantKv * plantKv / (4.0 * detunedKa);
        double sqrt = (Math.sqrt(detunedKa) - Math.sqrt(discriminant)) / denominator;
        return sqrt * sqrt;
    }

    private void systemIdentification() {
        int n = times.size();
        if (n < 4) {
            throw new IllegalArgumentException("Failed calibration.");
        }

        int start = Math.max(0, n - SAMPLES);
        double sum = 0;
        for (int i = start; i < n; i++) sum += velocities.get(i);
        double a = sum / (n - start);
        this.K = a / POWER;
        this.plantKv = 1.0 / K;

        List<Double> y = new ArrayList<>();
        List<Double> x = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            double vel = velocities.get(i) / POWER;
            if (vel > 0.8 * K || vel < 0.1 * K) continue;
            y.add(Math.log(K - vel));
            x.add(times.get(i));
        }
        double[] linReg = linearFit(x.stream().toArray(Double[]::new), y.stream().toArray(Double[]::new));
        if (linReg[1] == 0) throw new IllegalArgumentException("Failed calibration.");
        this.tau = -1.0 / linReg[1];
        this.plantKa = tau / K;
    }

    public double[] linearFit(Double[] x, Double[] y) {
        int n = x.length;
        double sumX = 0, sumXY = 0, sumY = 0, sumX2 = 0;
        for (int i = 0; i < n; i++) {
            sumX += x[i];
            sumY += y[i];
            sumXY += x[i] * y[i];
            sumX2 += x[i] * x[i];
        }
        double m = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
        double b = (sumY - m * sumX) / n;
        return new double[] {b, m};
    }
}

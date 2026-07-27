package org.firstinspires.ftc.teamcode.pedroPathing.control;

import android.annotation.SuppressLint;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Designs heading PID after plant identification. Foresight and ManualDrive heading
 * lock use derivative-on-measurement ({@code -kD · ω}), so suggested kD is applied to
 * measured angular velocity, not finite-differenced heading error.
 * Optional: {@code Controller.pid(kP, kI, kD).iZone(...).maxIntegral(...)} when enabling integral.
 */
@TeleOp(name = "Heading Auto Tuner", group = "3")
public class HeadingAutoTuner extends OpMode {
    private static final double ALPHA_LARGE = 0.6;
    private static final double ALPHA_SMALL = 0.9;
    private static final double BETA = 1.0;
    private static final double POWER = 0.6;
    private static final double RUNTIME = 3;
    private static final int SAMPLES = 15;

    private double tau;
    private double lambda_small;
    private double lambda_large;
    private double K;
    private final List<Double> times = new ArrayList<>();
    private final List<Double> angularVelocities = new ArrayList<>();
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
        telemetry.addLine("Group 3: heading feedback (run after group 2 plant ID).");
        telemetry.addLine("Turns in place for " + RUNTIME + " s. Leave room.");
        telemetry.addLine("Put kP/kD on foresightConfig.headingController (DoM uses measured ω).");
        telemetry.addLine("If using kI, add .iZone(...) and .maxIntegral(...) on the PID.");
        telemetry.update();
        follower.update();
    }

    @Override
    public void start() {
        timer.reset();
        lastTime = timer.seconds();
        follower.manual(0, 0, POWER);
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
            angularVelocities.add(Math.abs(follower.velocity().omega));
            telemetry.addData("angular velocity (rad/s)",
                    String.format(Locale.US, "%.4f", angularVelocities.get(angularVelocities.size() - 1)));

            if (timer.seconds() >= RUNTIME) {
                done = true;
                systemIdentification();
                follower.manual(0, 0, 0);
            } else {
                follower.manual(0, 0, POWER);
                telemetry.update();
                return;
            }
        }

        lambda_small = tau * ALPHA_SMALL;
        lambda_large = tau * ALPHA_LARGE;

        double kDLarge = getkD(lambda_large);
        double kPLarge = getkP(lambda_large);
        double kDSmall = getkD(lambda_small);
        double kPSmall = getkP(lambda_small);
        double feedforward = BETA / K;

        telemetry.addLine("--- headingController (DoM D on measured ω) ---");
        telemetry.addData("Large", String.format(Locale.US, "kP=%.4f, kD=%.4f", kPLarge, kDLarge));
        telemetry.addData("Small", String.format(Locale.US, "kP=%.4f, kD=%.4f", kPSmall, kDSmall));
        telemetry.addData("headingFeedforward k", String.format(Locale.US, "%.4f", feedforward));
        telemetry.addData("Est tau (s)", String.format(Locale.US, "%.4f", tau));
        telemetry.addData("Est K (rad/s per power)", String.format(Locale.US, "%.4f", K));
        telemetry.update();
    }

    private double getkP(double lambda) {
        return tau / (K * lambda * lambda);
    }

    private double getkD(double lambda) {
        return 1 / K * (2 * tau / lambda - 1);
    }

    private void systemIdentification() {
        int n = times.size();
        if (n < 4) {
            throw new IllegalArgumentException("Failed calibration.");
        }

        int start = Math.max(0, n - SAMPLES);
        double sum = 0;
        for (int i = start; i < n; i++) sum += angularVelocities.get(i);
        this.K = (sum / (n - start)) / POWER;

        List<Double> y = new ArrayList<>();
        List<Double> x = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            double vel = angularVelocities.get(i) / POWER;
            if (vel > 0.8 * K || vel < 0.1 * K) continue;
            y.add(Math.log(K - vel));
            x.add(times.get(i));
        }
        double[] linReg = linearFit(x.stream().toArray(Double[]::new), y.stream().toArray(Double[]::new));
        if (linReg[1] == 0) throw new IllegalArgumentException("Failed calibration.");
        this.tau = -1.0 / linReg[1];
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

package org.firstinspires.ftc.teamcode.pedroPathing.identification;

import android.annotation.SuppressLint;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Identifies anisotropic plant gains for one robot axis (forward/x or strafe/y).
 *
 * <p>Phase 1: multi-power steady-state fit {@code |u| = kS + kV·|v|}.
 * Phase 2: constant-power step for {@code kA = tau / K} (unscaled plant acceleration gain).
 *
 * <p>Copy results into {@code Constants.foresightConfig} as {@code kS_x/kV_x/kA_x} or
 * {@code kS_y/kV_y/kA_y}. Plant {@code kA} multiplies geometric centripetal acceleration only;
 * tangential schedule accel uses {@code brakeAccelFeedforward}. Run group 3 feedback tuners after
 * plant gains are set.
 */
abstract class AxisPlantIdentification extends OpMode {
    /** Steady-state commanded powers (magnitude). Direction alternates each step. */
    public static double[] STEADY_POWERS = {0.25, 0.35, 0.45, 0.55, 0.65, 0.75};
    public static double STEADY_DRIVE_S = 1.25;
    public static int STEADY_SAMPLE_COUNT = 25;
    public static double STEP_POWER = 0.4;
    public static double STEP_RUNTIME_S = 1.2;
    public static int STEP_STEADY_SAMPLES = 15;

    private enum Phase {
        STEADY,
        STEP,
        DONE
    }

    private final List<double[]> steadyPoints = new ArrayList<>();
    private final List<Double> stepTimes = new ArrayList<>();
    private final List<Double> stepSpeeds = new ArrayList<>();
    private final List<Double> recentSpeeds = new ArrayList<>();

    private final Timer timer = new Timer();
    private Follower follower;
    private Phase phase = Phase.STEADY;
    private int steadyIndex;
    private String failure;
    private PlantIdentificationMath.SteadyFit steadyFit;
    private PlantIdentificationMath.StepFit stepFit;

    /** {@code "x"} or {@code "y"} for Constants field names. */
    protected abstract String axisSuffix();

    /** Body-axis speed magnitude in length/s (forward or lateral). */
    protected abstract double axisSpeed(Follower follower);

    /** Apply signed power on this axis only. */
    protected abstract void applyPower(Follower follower, double signedPower);

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(Pose.zero());
        follower.update();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Plant ID (" + axisSuffix() + "): multi-power steady kS/kV, then step kA.");
        telemetry.addLine("Leave room — motion alternates direction between power steps.");
        telemetry.addLine("After finish, set kS_" + axisSuffix() + " / kV_" + axisSuffix()
                + " / kA_" + axisSuffix() + " in Constants.foresightConfig.");
        telemetry.addLine("Plant kA is for curve centripetal FF, not brake stop authority.");
        telemetry.addLine("Then re-run group 3 feedback auto-tuners.");
        telemetry.update();
        follower.update();
    }

    @Override
    public void start() {
        steadyPoints.clear();
        stepTimes.clear();
        stepSpeeds.clear();
        recentSpeeds.clear();
        steadyIndex = 0;
        phase = Phase.STEADY;
        failure = null;
        steadyFit = null;
        stepFit = null;
        timer.reset();
        beginSteadyStep();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        follower.update();

        if (failure != null) {
            applyPower(follower, 0);
            telemetry.addLine("FAILED: " + failure);
            telemetry.update();
            return;
        }

        switch (phase) {
            case STEADY:
                runSteady();
                break;
            case STEP:
                runStep();
                break;
            case DONE:
                applyPower(follower, 0);
                reportDone();
                break;
        }
        telemetry.update();
    }

    private void beginSteadyStep() {
        recentSpeeds.clear();
        timer.reset();
        double power = STEADY_POWERS[steadyIndex];
        double signed = ((steadyIndex % 2 == 0) ? 1.0 : -1.0) * power;
        applyPower(follower, signed);
        telemetry.addLine(String.format(Locale.US, "Steady %d/%d @ power=%.2f",
                steadyIndex + 1, STEADY_POWERS.length, power));
    }

    private void runSteady() {
        double power = STEADY_POWERS[steadyIndex];
        double signed = ((steadyIndex % 2 == 0) ? 1.0 : -1.0) * power;
        applyPower(follower, signed);

        double speed = Math.abs(axisSpeed(follower));
        recentSpeeds.add(speed);
        if (recentSpeeds.size() > STEADY_SAMPLE_COUNT) {
            recentSpeeds.remove(0);
        }

        telemetry.addData("phase", "steady");
        telemetry.addData("step", (steadyIndex + 1) + "/" + STEADY_POWERS.length);
        telemetry.addData("power", power);
        telemetry.addData("speed", speed);
        telemetry.addData("t", timer.seconds());

        if (timer.seconds() < STEADY_DRIVE_S) {
            return;
        }
        if (recentSpeeds.size() < STEADY_SAMPLE_COUNT) {
            return;
        }

        double avg = 0;
        for (double s : recentSpeeds) avg += s;
        avg /= recentSpeeds.size();
        steadyPoints.add(new double[] {avg, power});

        steadyIndex++;
        if (steadyIndex < STEADY_POWERS.length) {
            beginSteadyStep();
            return;
        }

        try {
            steadyFit = PlantIdentificationMath.fitSteadyState(steadyPoints);
        } catch (IllegalArgumentException e) {
            failure = e.getMessage();
            phase = Phase.DONE;
            return;
        }

        // Step response for kA.
        stepTimes.clear();
        stepSpeeds.clear();
        phase = Phase.STEP;
        timer.reset();
        applyPower(follower, STEP_POWER);
    }

    private void runStep() {
        applyPower(follower, STEP_POWER);
        double t = timer.seconds();
        double speed = Math.abs(axisSpeed(follower));
        stepTimes.add(t);
        stepSpeeds.add(speed);

        telemetry.addData("phase", "step");
        telemetry.addData("power", STEP_POWER);
        telemetry.addData("speed", speed);
        telemetry.addData("t", t);

        if (t < STEP_RUNTIME_S) {
            return;
        }

        applyPower(follower, 0);
        try {
            stepFit = PlantIdentificationMath.fitStepResponse(
                    stepTimes, stepSpeeds, STEP_POWER, STEP_STEADY_SAMPLES);
        } catch (IllegalArgumentException e) {
            failure = e.getMessage();
        }
        phase = Phase.DONE;
    }

    private void reportDone() {
        telemetry.addLine("Plant identification complete (" + axisSuffix() + ")");
        telemetry.addLine("Put these on Constants.foresightConfig:");

        if (steadyFit != null) {
            telemetry.addData("kS_" + axisSuffix() + " (steady)",
                    String.format(Locale.US, "%.5f", steadyFit.kS));
            telemetry.addData("kV_" + axisSuffix() + " (steady)",
                    String.format(Locale.US, "%.5f", steadyFit.kV));
        }
        if (stepFit != null) {
            telemetry.addData("kV_" + axisSuffix() + " (step 1/K)",
                    String.format(Locale.US, "%.5f", stepFit.kV));
            telemetry.addData("kA_" + axisSuffix() + " (plant tau/K)",
                    String.format(Locale.US, "%.5f", stepFit.kA));
            telemetry.addData("K (in/s per power)", String.format(Locale.US, "%.4f", stepFit.K));
            telemetry.addData("tau (s)", String.format(Locale.US, "%.4f", stepFit.tau));
            telemetry.addLine("Prefer steady kS/kV; use step kA for plant kA (unscaled).");
        }

        if (steadyFit != null && stepFit != null) {
            telemetry.addLine(String.format(Locale.US,
                    "Suggested: kS_%s=%.5f; kV_%s=%.5f; kA_%s=%.5f",
                    axisSuffix(), steadyFit.kS,
                    axisSuffix(), steadyFit.kV,
                    axisSuffix(), stepFit.kA));
        }

        for (int i = 0; i < steadyPoints.size(); i++) {
            double[] p = steadyPoints.get(i);
            telemetry.addData("steady[" + i + "]",
                    String.format(Locale.US, "v=%.3f u=%.2f", p[0], p[1]));
        }
    }
}

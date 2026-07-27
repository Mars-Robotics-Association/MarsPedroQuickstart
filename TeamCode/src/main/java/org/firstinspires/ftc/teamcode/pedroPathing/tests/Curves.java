package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import static com.pedropathing.api.Paths.curve;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.foresightConfig;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Curve stress test. Centripetal plant feedforward needs {@code kA_y} (and friends) in Constants.
 * For schedule slowdown on curvature, set {@link #LIMIT_LATERAL_ACCELERATION} or
 * {@code maxLateralAcceleration} in Constants (default ∞ disables curvature limiting).
 */
@TeleOp(name = "Curves", group = "4")
public class Curves extends OpMode {
    public static double DISTANCE = 48;
    public static double LIMIT_VELOCITY = 0;
    public static double LIMIT_ACCELERATION = 0;
    /** Most useful on curves: caps speed via {@code v ≤ √(a_lat / |κ|)}. */
    public static double LIMIT_LATERAL_ACCELERATION = 0;

    public double loops = 0, lastLoop = 0, loopTime = 0;
    private Path forwards, backwards;
    private boolean forward;
    private Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(72, 72, 0));
        telemetry.addLine("Group 4 curves. Plant kA (esp. kA_y) for centripetal FF.");
        telemetry.addLine("Set LIMIT_LATERAL_ACCELERATION > 0 so the schedule slows on curvature.");
        telemetry.addLine("Optional LIMIT_VELOCITY / LIMIT_ACCELERATION also shape v*(s).");
        telemetry.update();
    }

    @Override
    public void start() {
        forwards = applyLimits(curve(
                new Pose(72, 72),
                new Pose(Math.abs(DISTANCE) + 72, 72),
                new Pose(Math.abs(DISTANCE) + 72, DISTANCE + 72))
                .heading(new Interpolator() {
                    @Override
                    public double interpolate(Curve curve, double t) {
                        return Math.PI;
                    }

                    @Override
                    public double differentiate(Curve curve, double t) {
                        return 0;
                    }
                }));
        backwards = applyLimits(curve(
                new Pose(Math.abs(DISTANCE) + 72, DISTANCE + 72),
                new Pose(Math.abs(DISTANCE) + 72, 72),
                new Pose(72, 72))
                .heading(Interpolator.piecewise()
                        .until(0.5, Interpolator.tangent)
                        .until(1.0, Interpolator.constant(0))));
        follower.follow(forwards);
    }

    private Path applyLimits(Path path) {
        if (LIMIT_VELOCITY > 0) {
            path = path.with(foresightConfig.limitVelocity(LIMIT_VELOCITY));
        }
        if (LIMIT_ACCELERATION > 0) {
            path = path.with(foresightConfig.limitAcceleration(LIMIT_ACCELERATION));
        }
        if (LIMIT_LATERAL_ACCELERATION > 0) {
            path = path.with(foresightConfig.limitLateralAcceleration(LIMIT_LATERAL_ACCELERATION));
        }
        return path;
    }

    @Override
    public void loop() {
        loops++;

        if (loops > 10) {
            double now = System.currentTimeMillis();
            loopTime = (now - lastLoop) / loops;
            lastLoop = now;
            loops = 0;
        }

        double nanoBefore = System.nanoTime();

        follower.update();

        telemetry.addData("Calculation Nano Time", System.nanoTime() - nanoBefore);
        telemetry.addData("Calculation Ms", 1e-6 * (System.nanoTime() - nanoBefore));

        Foresight foresight = (Foresight) follower.getAlgorithm();

        if (follower.atParametricEnd()) {
            if (forward) {
                follower.follow(backwards);
            } else {
                follower.follow(forwards);
            }
            forward = !forward;
        }

        telemetry.addData("plannedVelocity", foresight.getPlannedVelocity());
        telemetry.addData("plannedAcceleration", foresight.getPlannedAcceleration());
        telemetry.addData("targetVelocity", foresight.getTargetVelocity());
        telemetry.addData("tangentialAccel", foresight.getTangentialAccel());
        telemetry.addData("LIMIT_VELOCITY", LIMIT_VELOCITY);
        telemetry.addData("LIMIT_ACCELERATION", LIMIT_ACCELERATION);
        telemetry.addData("LIMIT_LATERAL_ACCELERATION", LIMIT_LATERAL_ACCELERATION);
        telemetry.addData("Loop Time Hz", 1000 / loopTime);
        telemetry.addData("Mode", follower.mode());
        telemetry.addData("Following?", follower.following());
        telemetry.addData("Pose", follower.pose());
        telemetry.update();
    }
}

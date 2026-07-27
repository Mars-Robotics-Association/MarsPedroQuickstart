package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.foresightConfig;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;

/**
 * Compound curve + line path. Optional per-segment motion limits feed the arc-length schedule.
 */
@TeleOp(name = "Compound Curve", group = "4")
public class CompoundCurve extends OpMode {
    public static double DISTANCE = 48;
    public static double LIMIT_VELOCITY = 0;
    public static double LIMIT_ACCELERATION = 0;
    public static double LIMIT_LATERAL_ACCELERATION = 0;

    public double loops = 0, lastLoop = 0, loopTime = 0;
    private Path path;
    private boolean forward;
    private Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(72, 72, 0));
        telemetry.addLine("Group 4 compound path. Plant kA improves curve tracking.");
        telemetry.addLine("LIMIT_LATERAL_ACCELERATION slows the schedule on curvature.");
        telemetry.update();
    }

    @Override
    public void start() {
        Path line1 = applyLimits(curve(
                new Pose(72, 72),
                new Pose(Math.abs(DISTANCE) + 72, 72),
                new Pose(Math.abs(DISTANCE) + 72, DISTANCE + 72)).constant(0));
        Path line2 = applyLimits(line(
                new Pose(DISTANCE + 72, DISTANCE + 72, 0),
                new Pose(72, 72, 0)).linear(0, Math.PI));
        path = path(line1, line2);
        path = path(path, path).constant(0);
        follower.follow(path);
    }

    private Path applyLimits(Path p) {
        if (LIMIT_VELOCITY > 0) {
            p = p.with(foresightConfig.limitVelocity(LIMIT_VELOCITY));
        }
        if (LIMIT_ACCELERATION > 0) {
            p = p.with(foresightConfig.limitAcceleration(LIMIT_ACCELERATION));
        }
        if (LIMIT_LATERAL_ACCELERATION > 0) {
            p = p.with(foresightConfig.limitLateralAcceleration(LIMIT_LATERAL_ACCELERATION));
        }
        return p;
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

        if (follower.distanceToEndpoint() < 5 && foresight.testVelocity()) {
            follower.follow(path);
        }

        telemetry.addData("velocity", follower.velocity().toVector2D().x());
        telemetry.addData("plannedVelocity", foresight.getPlannedVelocity());
        telemetry.addData("plannedAcceleration", foresight.getPlannedAcceleration());
        telemetry.addData("targetVelocity", foresight.getTargetVelocity());
        telemetry.addData("tangentialAccel", foresight.getTangentialAccel());
        telemetry.addData("error", Math.max(follower.velocity().toVector2D().x() - foresight.getTargetVelocity(), 0));
        telemetry.addData("LIMIT_VELOCITY", LIMIT_VELOCITY);
        telemetry.addData("LIMIT_ACCELERATION", LIMIT_ACCELERATION);
        telemetry.addData("LIMIT_LATERAL_ACCELERATION", LIMIT_LATERAL_ACCELERATION);

        telemetry.addData("Loop Time Hz", 1000 / loopTime);
        telemetry.addData("Mode", follower.mode());
        telemetry.addData("Following?", follower.following());
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("Parametric End", follower.atParametricEnd());
        telemetry.addData("TranslationalError", foresight.getTranslationalError());
        telemetry.addData("Velocity", foresight.testVelocity());
        telemetry.addData("Distance", follower.distanceToEndpoint());
        telemetry.addData("Pose", follower.pose());
        telemetry.addData("forward", forward);
        telemetry.addData("power", Arrays.toString(((Mecanum) follower.drivetrain).wheelPowers));
        telemetry.update();
    }
}

package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import static com.pedropathing.api.Paths.line;
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
 * Back-and-forth line stress test. Demonstrates per-path motion limit modifiers (disabled when
 * the corresponding static is non-positive). Limits feed the always-on arc-length schedule
 * {@code v*(s)}, not only an online coast clamp.
 */
@TeleOp(name = "Line Test", group = "4")
public class LineTest extends OpMode {
    public static double DISTANCE = 48;
    /** Set &gt; 0 to cap path speed (in/s) via foresightConfig.limitVelocity. */
    public static double LIMIT_VELOCITY = 0;
    /** Set &gt; 0 to cap path acceleration via foresightConfig.limitAcceleration. */
    public static double LIMIT_ACCELERATION = 0;
    /** Set &gt; 0 to curvature-limit speed via foresightConfig.limitLateralAcceleration. */
    public static double LIMIT_LATERAL_ACCELERATION = 0;

    public double loops = 0, lastLoop = 0, loopTime = 0;
    private Path line1, line2;
    private boolean forward;
    private Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(72, 72, 0));
        telemetry.addLine("Group 4 path test. Optional LIMIT_VELOCITY / LIMIT_ACCELERATION /");
        telemetry.addLine("LIMIT_LATERAL_ACCELERATION > 0 shape the v*(s) schedule.");
        telemetry.update();
    }

    @Override
    public void start() {
        line1 = applyLimits(line(new Pose(72, 72, 0), new Pose(DISTANCE + 72, 72, 0)).constant(0));
        line2 = applyLimits(line(new Pose(DISTANCE + 72, 72, 0), new Pose(72, 72, 0)).constant(0));
        follower.follow(line1);
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

        if (follower.distanceToEndpoint() < 5 && foresight.testVelocity()) {
            if (forward) {
                follower.follow(line2);
            } else {
                follower.follow(line1);
            }
            forward = !forward;
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

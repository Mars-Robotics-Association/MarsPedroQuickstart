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
 * Lateral line stress test with optional per-path motion limits that feed the arc-length schedule.
 */
@TeleOp(name = "Strafe Line Test", group = "4")
public class StrafeLineTest extends OpMode {
    public static double DISTANCE = 48;
    public static double LIMIT_VELOCITY = 0;
    public static double LIMIT_ACCELERATION = 0;
    public static double LIMIT_LATERAL_ACCELERATION = 0;

    public double loops = 0, lastLoop = 0, loopTime = 0;
    private Path line1, line2;
    private boolean forward;
    private Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(72, 72, 0));
        telemetry.addLine("Group 4 lateral path test. Optional LIMIT_VELOCITY /");
        telemetry.addLine("LIMIT_ACCELERATION / LIMIT_LATERAL_ACCELERATION shape v*(s).");
        telemetry.update();
    }

    @Override
    public void start() {
        // Lateral lines in field y (robot starts heading 0).
        line1 = applyLimits(line(new Pose(72, 72, 0), new Pose(72, DISTANCE + 72, 0)).constant(0));
        line2 = applyLimits(line(new Pose(72, DISTANCE + 72, 0), new Pose(72, 72, 0)).constant(0));
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

        telemetry.addData("velocity", follower.velocity().toVector2D().y());
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

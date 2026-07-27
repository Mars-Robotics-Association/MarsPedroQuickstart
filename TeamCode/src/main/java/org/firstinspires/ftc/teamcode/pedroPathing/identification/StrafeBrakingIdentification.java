package org.firstinspires.ftc.teamcode.pedroPathing.identification;

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

/**
 * Strafe (robot-y) braking profile identification for anisotropic
 * {@code linearBrakeCoefficients(1,1)} / {@code quadraticBrakeCoefficients(1,1)}.
 */
@TeleOp(name = "Strafe Braking Identification", group = "2")
public class StrafeBrakingIdentification extends OpMode {
    private static final double[] TEST_POWERS =
            {1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2};
    private static final double BRAKING_POWER = 0.2;
    private static final int DRIVE_TIME_MS = 1000;

    private enum State {
        START_MOVE,
        WAIT_DRIVE_TIME,
        APPLY_BRAKE,
        WAIT_BRAKE_TIME,
        RECORD,
        DONE
    }

    private static class BrakeRecord {
        final double timeMs;
        final Pose pose;
        final double velocity;

        BrakeRecord(double timeMs, Pose pose, double velocity) {
            this.timeMs = timeMs;
            this.pose = pose;
            this.velocity = velocity;
        }
    }

    private State state = State.START_MOVE;
    private final Timer timer = new Timer();
    private int iteration = 0;
    private Vector2D startPosition;
    private double measuredVelocity;
    private final List<double[]> velocityToBrakingDistance = new ArrayList<>();
    private final List<BrakeRecord> brakeData = new ArrayList<>();
    private Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(Pose.zero());
        follower.update();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Strafe braking ID: left/right runs, then reverse-power brake.");
        telemetry.addLine("Use results for Matrix.diag entries at (1,1) — strafe axis.");
        telemetry.addLine("Pair with Forward Braking Identification for anisotropic brakes.");
        telemetry.addLine("Leave at least 4-5 feet clear.");
        telemetry.update();
        follower.update();
    }

    @Override
    public void start() {
        timer.reset();
        follower.update();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        follower.update();

        double direction = (iteration % 2 == 0) ? 1 : -1;

        switch (state) {
            case START_MOVE: {
                if (iteration >= TEST_POWERS.length) {
                    state = State.DONE;
                    break;
                }

                double currentPower = TEST_POWERS[iteration];
                follower.manual(0, direction * currentPower, 0);

                timer.reset();
                state = State.WAIT_DRIVE_TIME;
                break;
            }

            case WAIT_DRIVE_TIME: {
                if (timer.milliseconds() >= DRIVE_TIME_MS) {
                    measuredVelocity = Math.abs(lateralVelocity());
                    startPosition = follower.pose().toVector2D();
                    state = State.APPLY_BRAKE;
                }
                break;
            }

            case APPLY_BRAKE: {
                // Same pattern as ForwardBrakingIdentification: fixed low power while measuring stop.
                follower.manual(0, BRAKING_POWER * direction, 0);
                timer.reset();
                state = State.WAIT_BRAKE_TIME;
                break;
            }

            case WAIT_BRAKE_TIME: {
                double t = timer.milliseconds();
                Pose currentPose = follower.pose();
                double currentVelocity = Math.abs(lateralVelocity());
                brakeData.add(new BrakeRecord(t, currentPose, currentVelocity));

                Vector2D lateralUnit = Vector2D.polar(direction, follower.pose().heading() + Math.PI / 2);
                if (follower.velocity().toVector2D().dot(lateralUnit) <= 0) {
                    state = State.RECORD;
                }
                break;
            }

            case RECORD: {
                Vector2D endPosition = follower.pose().toVector2D();
                double brakingDistance = endPosition.minus(startPosition).magnitude();
                velocityToBrakingDistance.add(new double[] {measuredVelocity, brakingDistance});

                telemetry.addLine(String.format("Test %d: v=%.3f  d=%.3f",
                        iteration, measuredVelocity, brakingDistance));

                iteration++;
                state = State.START_MOVE;
                break;
            }

            case DONE: {
                follower.manual(0, 0, 0);
                double[] coefficients = ForwardBrakingIdentification.quadraticFit(velocityToBrakingDistance);

                telemetry.addLine("Tuning Complete (strafe)");
                telemetry.addLine("Set Constants diagonal (1,1) entries:");
                telemetry.addData("Strafe Linear Brake Coefficient", coefficients[0]);
                telemetry.addData("Strafe Quadratic Brake Coefficient", coefficients[1]);
                for (BrakeRecord record : brakeData) {
                    Pose p = record.pose;
                    telemetry.addLine(String.format("t=%.0f ms, x=%.2f, y=%.2f, θ=%.2f, v=%.2f",
                            record.timeMs, p.x(), p.y(), p.heading(), record.velocity));
                }
                break;
            }
        }
        telemetry.update();
    }

    private double lateralVelocity() {
        return follower.velocity().toVector2D()
                .dot(Vector2D.polar(1, follower.pose().heading() + Math.PI / 2));
    }
}

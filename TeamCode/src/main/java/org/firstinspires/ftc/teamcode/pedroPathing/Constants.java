package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Pinpoint;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Robot hardware and Foresight constants.
 *
 * <p>Suggested tuning order (DS TeleOp groups):
 * <ol>
 *   <li>Group 1 localization — offsets / LocalizationTest</li>
 *   <li>Group 2 identification — max vel/decel, forward+strafe braking, forward+strafe plant</li>
 *   <li>Group 3 control — translational / heading auto-tuners (after plant gains are set)</li>
 *   <li>Group 4 path tests — lines/curves; optional per-path limitVelocity/limitAcceleration</li>
 * </ol>
 *
 * <p>Plant model (robot frame): {@code u = kS·sign(v) + kV·v + kA·a_des}.
 * Path centripetal is geometric {@code a_n = centripetalScaling · v² · κ} (default scale 1);
 * power comes from plant {@code kA}, not {@code robotMass}.
 */
public class Constants {
    public static MecanumConfig mecanumConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("lf");
                c.backLeftName.set("lb");
                c.frontRightName.set("rf");
                c.backRightName.set("rb");
                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.manualBrakeMode.set(true);
            }
    );

    public static PinpointConfig pinpointConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.xPodOffset.set(0.0);
                c.yPodOffset.set(0.0);
                c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            }
    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                // --- Group 2: max speed / free deceleration ---
                c.maxAchievableForwardVelocity.set(81.175);
                c.maxAchievableStrafeVelocity.set(66.8431);
                c.maxAchievableForwardDeceleration.set(30.3333);
                c.maxAchievableStrafeDeceleration.set(62.58098);

                // --- Group 2: anisotropic coast/brake matrices (diag: x=forward, y=strafe) ---
                // ForwardBrakingIdentification -> (0,0); StrafeBrakingIdentification -> (1,1).
                c.linearBrakeCoefficients.set(Matrix.diag(0.0788, 0.0788));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.00191035, 0.00191035));

                // --- Group 2: anisotropic plant feedforward (defaults 0 = disabled) ---
                // ForwardPlantIdentification -> *_x; StrafePlantIdentification -> *_y.
                // Prefer steady-state kS/kV and step-response kA from those opmodes.
                c.kS_x.set(0.0);
                c.kV_x.set(0.0);
                c.kA_x.set(0.0);
                c.kS_y.set(0.0);
                c.kV_y.set(0.0);
                c.kA_y.set(0.0);
                // Geometric curve feedforward scale; leave 1.0 unless intentionally reducing a_n.
                c.centripetalScaling.set(1.0);

                // --- Group 3: feedback (retune after plant gains are non-zero) ---
                c.brakeController.set(Controller.pid(0.2, 0, 0));
                c.headingController.set(Controller.pid(2, 0, 0.01));
                // Optional after auto-tuners:
                // c.forwardTranslationalController.set(Controller.pid(kP, 0, 0));
                // c.lateralTranslationalController.set(Controller.pid(kP, 0, 0));
                // c.headingController.set(Controller.pid(kP, 0, kD).iZone(...).maxIntegral(...));
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new Pinpoint(h, pinpointConfig),
                new Mecanum(h, mecanumConfig),
                new Foresight(foresightConfig));
    }
}

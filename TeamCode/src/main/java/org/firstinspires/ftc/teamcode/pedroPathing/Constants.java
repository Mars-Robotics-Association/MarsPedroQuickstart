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
 * Robot hardware and Foresight constants (PedroPathing {@code mars3-v1.1.0}+).
 *
 * <p>Suggested tuning order (DS TeleOp groups):
 * <ol>
 *   <li>Group 1 localization — offsets / LocalizationTest</li>
 *   <li>Group 2 identification — max vel/decel, forward+strafe braking, forward+strafe plant</li>
 *   <li>Group 3 control — translational / heading auto-tuners (after plant gains are set)</li>
 *   <li>Group 4 path tests — lines/curves; optional per-path limitVelocity / limitAcceleration /
 *       limitLateralAcceleration (feed the always-on arc-length schedule {@code v*(s)})</li>
 * </ol>
 *
 * <p>Plant model (robot frame): {@code u = kS·sign(v) + kV·v + kA·a_n}, where
 * {@code a_n = centripetalScaling · v_τ² · κ} is geometric centripetal only. Tangential schedule
 * acceleration is handled by {@code brakeAccelFeedforward}, not plant {@code kA}.
 *
 * <p>Foresight always plans and tracks an arc-length velocity profile. Max vel/decel, path limits,
 * and (when set) {@code maxLateralAcceleration} shape that schedule; brake coefficients still
 * provide a reactive remaining-distance clamp under the plan.
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
                // --- Group 2: max speed / free deceleration (shape the v*(s) profile) ---
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
                // Plant kA multiplies centripetal a_n only (see class javadoc).
                c.kS_x.set(0.0);
                c.kV_x.set(0.0);
                c.kA_x.set(0.0);
                c.kS_y.set(0.0);
                c.kV_y.set(0.0);
                c.kA_y.set(0.0);
                // Geometric curve feedforward scale; leave 1.0 unless intentionally reducing a_n.
                c.centripetalScaling.set(1.0);

                // --- Group 4 / profile: curvature speed limit v ≤ √(a_lat / |κ|) ---
                // Infinity disables. Set a finite value (or use path.limitLateralAcceleration) so
                // curves slow on high curvature; otherwise only plant kA provides curve authority.
                // c.maxLateralAcceleration.set(80.0);
                // Optional absolute |κ| clamp for plant FF + profile limits:
                // c.maxCurvature.set(0.5);

                // Reserve a fraction of unit drive power for heading/translational corrections
                // when the schedule is unconstrained (full-power coast). Default 0.
                // c.correctionPowerReserve.set(0.1);

                // --- Group 3: feedback (retune after plant gains are non-zero) ---
                c.brakeController.set(Controller.pid(0.2, 0, 0));
                c.headingController.set(Controller.pid(2, 0, 0.01));
                // Optional after auto-tuners:
                // c.forwardTranslationalController.set(Controller.pid(kP, 0, 0));
                // c.lateralTranslationalController.set(Controller.pid(kP, 0, 0));
                // c.headingController.set(Controller.pid(kP, 0, kD).iZone(...).maxIntegral(...));

                // --- Advanced (defaults are usually fine) ---
                // Lookahead on drive tangent / heading (0 = closest-point only):
                // c.lookaheadTime.set(0.15);
                // c.lookaheadMinDistance.set(2.0);
                // c.lookaheadMaxDistance.set(12.0);
                // Deviation / overspeed replan (segment changes always replan):
                // c.replanTranslationalThreshold.set(5.0);
                // c.replanHeadingThreshold.set(Math.toRadians(25.0));
                // c.replanHoldCycles.set(3);
                // c.replanOverspeedRatio.set(0.15);
                // c.replanOverspeedAbsolute.set(2.0);
                // c.replanCooldown.set(0.1);
                // c.velocityProfileSamples.set(48);
                // c.useFullEndConstraints.set(false);
                // Tangential accel biquad for DoM damping (Bessel defaults: 10 Hz, Q=1/√3):
                // c.tangentialAccelFilterCutoffHz.set(10.0);
                // c.tangentialAccelFilterQ.set(1.0 / Math.sqrt(3.0));
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new Pinpoint(h, pinpointConfig),
                new Mecanum(h, mecanumConfig),
                new Foresight(foresightConfig));
    }
}

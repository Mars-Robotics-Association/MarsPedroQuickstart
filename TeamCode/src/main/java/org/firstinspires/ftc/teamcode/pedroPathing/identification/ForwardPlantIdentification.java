package org.firstinspires.ftc.teamcode.pedroPathing.identification;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Vector2D;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Robot-x (forward) plant identification for {@code kS_x}, {@code kV_x}, {@code kA_x}.
 */
@TeleOp(name = "Forward Plant Identification", group = "2")
public class ForwardPlantIdentification extends AxisPlantIdentification {
    @Override
    protected String axisSuffix() {
        return "x";
    }

    @Override
    protected double axisSpeed(Follower follower) {
        return follower.velocity().toVector2D()
                .dot(Vector2D.polar(1, follower.pose().heading()));
    }

    @Override
    protected void applyPower(Follower follower, double signedPower) {
        follower.manual(signedPower, 0, 0);
    }
}

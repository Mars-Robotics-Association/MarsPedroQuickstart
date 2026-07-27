package org.firstinspires.ftc.teamcode.pedroPathing.identification;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Vector2D;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Robot-y (strafe) plant identification for {@code kS_y}, {@code kV_y}, {@code kA_y}.
 * Mecanum typically needs higher y-axis plant gains than x.
 */
@TeleOp(name = "Strafe Plant Identification", group = "2")
public class StrafePlantIdentification extends AxisPlantIdentification {
    @Override
    protected String axisSuffix() {
        return "y";
    }

    @Override
    protected double axisSpeed(Follower follower) {
        return follower.velocity().toVector2D()
                .dot(Vector2D.polar(1, follower.pose().heading() + Math.PI / 2));
    }

    @Override
    protected void applyPower(Follower follower, double signedPower) {
        follower.manual(0, signedPower, 0);
    }
}

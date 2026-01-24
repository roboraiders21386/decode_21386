package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

@Configurable
public class ShooterConstants {
    public static Pose redgoal = new Pose(138,138);
    public static Pose bluegoal = redgoal.mirror();
    public static double scoreheight = 26;//inches
    public static double scoreangle = Math.toRadians(-30);//radians
    public static double PASS_THROUGH_POINT_RADIUS = 5;//inches

   /* public static double getFlywheelTicksFromVelocity(double velocity){
        return MathFunctions.clamp(94.501*velocity/12-187.96+flywheelOffset, FLYWHEEL_MIN_SPEED, FLYWHEEL_MAX_SPEED);
    }

    */

    public static double getHoodTicksFromDegrees(double degrees){
        return 0.0226 * degrees - 0.7443;
    }
}

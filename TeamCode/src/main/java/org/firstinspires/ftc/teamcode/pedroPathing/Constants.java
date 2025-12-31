package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .translationalPIDFCoefficients(new PIDFCoefficients(0.07,0,0,0))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8,0.1,0.3,0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.2, 0.2, 0.00003, 0.01, 0.6))
            .forwardZeroPowerAcceleration(-44.885)
            .lateralZeroPowerAcceleration(-85.022)
            .mass(9.62);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.5)
            .xVelocity(59.9)
            .yVelocity(40)
            .rightFrontMotorName("RF")
            .rightRearMotorName("RB")
            .leftRearMotorName("LB")
            .leftFrontMotorName("LF")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .twoWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

    public static TwoWheelConstants localizerConstants = new  TwoWheelConstants()

            .forwardEncoder_HardwareMapName("LB")
            .strafeEncoder_HardwareMapName("LF")
            .IMU_HardwareMapName("imu")
            .forwardPodY(-2)
            .strafePodX(0)
            .forwardTicksToInches(0.0027) //11/30/2025 - 0.00204 current 12/28/2025
            .strafeTicksToInches(0.0024) //0.3, 1.6, 0.035 0.0019 0.07 0.0027
            .forwardEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE)
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            );
}

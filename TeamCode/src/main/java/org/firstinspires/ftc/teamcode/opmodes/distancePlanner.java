/*package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "pose", group = "Testing")
public class distancePlanner extends OpMode {
    private Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        //unknown starting pose
        follower.setStartingPose(new Pose(0, 0, 0));

    @Override
    public void loop() {
        // refresh odometry
        follower.update();

        // Get current pose
        Pose currentPose = follower.getPose();

        // Display coordinates
        telemetry.addData("X", currentPose.getX());
        telemetry.addData("Y", currentPose.getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(currentPose.getHeading()));
        telemetry.update();
    }
}

 */
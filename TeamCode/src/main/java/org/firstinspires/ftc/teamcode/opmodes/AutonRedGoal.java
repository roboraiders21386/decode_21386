package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "RUN RED GOAL", group = "Autonomous")
public class AutonRedGoal extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(120.000, 127.000, Math.toRadians(223));
    private final Pose pickupPose = new Pose(104.000, 84.000, Math.toRadians(90));

    private PathChain path1;

    public void buildPaths() {
        // Path 1: Start (120, 127) to Pickup Position (104, 84) with heading 90°
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pickupPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickupPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Move to pickup position
                follower.followPath(path1, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    // Reached pickup position (104, 84) at 90° heading
                    // Autonomous complete
                    setPathState(-1);
                }
                break;

            default:
                // Autonomous complete
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        pathState = 0;

        telemetry.addData("Status", "Red Goal Auto Initialized");
        telemetry.addData("Start", "X: %.1f, Y: %.1f, H: 223°",
                startPose.getX(), startPose.getY());
        telemetry.addData("Pickup Pose", "X: %.1f, Y: %.1f, H: 90°",
                pickupPose.getX(), pickupPose.getY());
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        if (opmodeTimer.getElapsedTimeSeconds() > 30) {
            requestOpModeStop();
        }

        telemetry.addData("Path State", pathState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Position", "X: %.1f, Y: %.1f",
                follower.getPose().getX(), follower.getPose().getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Time", "%.1f sec", opmodeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("Status", "Red Goal Auto Complete");
        telemetry.addData("Final Time", "%.2f seconds", opmodeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
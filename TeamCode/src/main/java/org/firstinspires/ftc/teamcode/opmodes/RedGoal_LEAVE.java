package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "do not use - RGL", group = "Autonomous")
public class RedGoal_LEAVE extends OpMode {

    private Follower follower;
    private Timer opmodeTimer;
    private int pathState;

    // RED Goal Starting Position and Park Position


    //Updated co-ordinates to be on the RED side
    private final Pose startPose = new Pose(128, 128, Math.toRadians(270));
    private final Pose parkPose = new Pose(96, 60.200, Math.toRadians(360));


    private PathChain parkPath;

    public void buildPaths() {
        // Single path: Start to Park (diagonal forward-right, maintaining heading)
        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, parkPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Move diagonally to park position
                follower.followPath(parkPath, true);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    // Park complete
                    pathState = -1;
                }
                break;

            default:
                // Autonomous complete
                break;
        }
    }

    @Override
    public void init() {
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Initialize Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        pathState = 0;

        telemetry.addData("Status", "Red Wall Park Only - Initialized");
        telemetry.addData("Start", "X: %.1f, Y: %.1f, H: 270°",
                startPose.getX(), startPose.getY());
        telemetry.addData("Park", "X: %.1f, Y: %.1f (ΔX: +17, ΔY: +15)",
                parkPose.getX(), parkPose.getY());
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
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
        telemetry.addData("Status", "Red Wall Park Complete");
        telemetry.addData("Final Time", "%.2f seconds", opmodeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
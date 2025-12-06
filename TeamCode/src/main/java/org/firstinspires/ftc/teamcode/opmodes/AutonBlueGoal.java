package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Goal Auto", group = "Autonomous")
public class AutonBlueGoal extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private int pathState;

    // Based on the diagram - coordinates adjusted to match field layout
    private final Pose startPose = new Pose(27.66, 128.38, Math.toRadians(315)); // Start position
    private final Pose scorePose = new Pose(53.21, 106.21, Math.toRadians(315)); // High basket scoring position
    private final Pose scanPose = new Pose(71.58, 119.09, Math.toRadians(270)); // Scan position for vision
    private final Pose prepToPickup = new Pose(40.75, 84.25, Math.toRadians(270)); // Prepare to pickup position
    private final Pose pickup1Pose = new Pose(10.98, 84.04, Math.toRadians(180)); // First sample pickup

    private final Pose parkPose = new Pose(48, 60, Math.toRadians(315));
    private Path scorePreload;
    private PathChain scanCode, gotoGrabPickup1, grabPickup1, scorePickup1;

    public void buildPaths() {
//        // Path 1: Score preloaded specimen on high chamber
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        // Path 2: Move to scan position for vision detection
        scanCode = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();

        // Path 3: Move to prepare for pickup position
        gotoGrabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scanPose, prepToPickup))
                .setLinearHeadingInterpolation(scanPose.getHeading(), prepToPickup.getHeading())
                .build();

        // Path 4: Move to first sample and grab it
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(prepToPickup, pickup1Pose))
                .setLinearHeadingInterpolation(prepToPickup.getHeading(), pickup1Pose.getHeading())
                .build();

        // Path 5: Return to scoring position with sample
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start: Score preloaded specimen
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                // Wait until robot reaches scoring position
                if (!follower.isBusy()) {
                    // TODO: Add code here to score specimen on high chamber
                    // e.g., lift.setPosition(HIGH_POSITION); claw.release();

                    follower.followPath(scanCode, true);
                    setPathState(2);
                }
                break;

            case 2:
                // Wait until robot reaches scan position
                if (!follower.isBusy()) {
                    // TODO: Add code here to scan for sample colors using vision
                    // e.g., processVisionData();

                    follower.followPath(gotoGrabPickup1, true);
                    setPathState(3);
                }
                break;

            case 3:
                // Wait until robot reaches prep position
                if (!follower.isBusy()) {
                    // TODO: Add code here to prepare intake/claw
                    // e.g., intake.setPower(1.0); arm.setPosition(PICKUP_POSITION);

                    follower.followPath(grabPickup1, true);
                    setPathState(4);
                }
                break;

            case 4:
                // Wait until robot reaches first sample
                if (!follower.isBusy()) {
                    // TODO: Add code here to grab the sample
                    // e.g., claw.close(); arm.setPosition(CARRY_POSITION);

                    follower.followPath(scorePickup1, true);
                    setPathState(5);
                }
                break;

            case 5:
                // Wait until robot returns to scoring position
                if (!follower.isBusy()) {
                    // TODO: Add code here to score sample in high basket
                    // e.g., lift.setPosition(HIGH_BASKET); claw.release();

                    // End autonomous
                    setPathState(-1);
                }
                break;

            default:
                // Autonomous complete - do nothing
                break;
        }
    }

    /** Change path state and reset timer **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        // Initialize timers
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Initialize follower with hardware map
        follower = Constants.createFollower(hardwareMap);

        // Build all paths
        buildPaths();

        // Set starting pose
        follower.setStartingPose(startPose);

        // Initialize path state
        pathState = 0;

        // TODO: Initialize your robot hardware here
        // e.g., lift = hardwareMap.get(DcMotor.class, "lift");
        //       claw = hardwareMap.get(Servo.class, "claw");

        telemetry.addData("Status", "Blue Goal Auto Initialized");
        telemetry.addData("Start Pose", "X: %.2f, Y: %.2f, Heading: %.2f°",
                startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
        telemetry.update();
    }

    @Override
    public void start() {
        // Reset timers when autonomous starts
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();

        // Ensure we start in state 0
        pathState = 0;
    }

    // Main loop - this is called continuously after pressing "Play"
    @Override
    public void loop() {
        // CRITICAL: These must be called every loop iteration
        follower.update();          // Updates Pedro Pathing follower
        autonomousPathUpdate();     // Updates our state machine

        // Add a timeout safety feature (30 seconds)
        if (opmodeTimer.getElapsedTimeSeconds() > 30) {
            requestOpModeStop();
        }

        // Telemetry for debugging
        telemetry.addData("Path State", pathState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("X Position", "%.2f", follower.getPose().getX());
        telemetry.addData("Y Position", "%.2f", follower.getPose().getY());
        telemetry.addData("Heading", "%.2f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Elapsed Time", "%.1f sec", opmodeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    @Override
    public void stop() {
        // Clean up when autonomous stops
        telemetry.addData("Status", "Blue Goal Auto Complete");
        telemetry.addData("Final Time", "%.2f seconds", opmodeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
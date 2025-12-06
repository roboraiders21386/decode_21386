package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "RUN BLUE WALL", group = "Autonomous")
public class AutonBlueWall extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer, actionTimer;
    private int pathState;

    // Hardware
    private DcMotor shooter;
    private DcMotor intake;
    private CRServo left_Transfer;
    private CRServo right_Transfer;

    // Blue Wall Starting Position and Poses
    private final Pose startPose = new Pose(72.000, 8.000, Math.toRadians(270));
    private final Pose scorePose = new Pose(72.000, 42.000, Math.toRadians(270));
    private final Pose shootPose = new Pose(72.000, 42.000, Math.toRadians(325));

    private PathChain path1, path2;

    public void buildPaths() {
        // Path 1: Start to Score Position (no rotation yet)
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        // Path 2: Rotate in place from 270° to 325° (counter-clockwise)
        // Using a tiny movement to ensure path executes
        Pose rotationHelper = new Pose(72.000, 42.001, Math.toRadians(325));
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, rotationHelper))
                .setLinearHeadingInterpolation(scorePose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start: Move to position (72, 42) maintaining 270° heading
                follower.followPath(path1, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    // Robot is at (72, 42) facing 270°
                    // Now rotate counter-clockwise to 325°
                    follower.followPath(path2, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    // Robot is now facing 325°
                    // Set shooter direction first, then power
                    shooter.setDirection(DcMotorSimple.Direction.FORWARD);
                    actionTimer.resetTimer();
                    setPathState(3);
                }
                break;

            case 3:
                // Spin up shooter first (wait a moment for direction to take effect)
                if (actionTimer.getElapsedTimeSeconds() > 0.1) {
                    shooter.setPower(0.8);
                }
                // Wait 3 seconds total, then start transfer while shooter continues
                if (actionTimer.getElapsedTimeSeconds() > 3.0) {
                    // Run transfer to feed specimen into shooter
                    left_Transfer.setPower(-1);
                    right_Transfer.setPower(1);
                    setPathState(4);
                }
                break;

            case 4:
                // Run both shooter and transfer for ~4 more seconds (7 total)
                if (actionTimer.getElapsedTimeSeconds() > 7.0) {
                    // Stop transfer and shooter
                    left_Transfer.setPower(0);
                    right_Transfer.setPower(0);
                    shooter.setPower(0);

                    // Autonomous complete
                    setPathState(-1);
                }
                break;

            default:
                // Autonomous complete - ensure everything is stopped
                shooter.setPower(0);
                intake.setPower(0);
                left_Transfer.setPower(0);
                right_Transfer.setPower(0);
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
        actionTimer = new Timer();
        opmodeTimer.resetTimer();

        // Initialize Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        pathState = 0;

        // Initialize hardware
        try {
            shooter = hardwareMap.get(DcMotor.class, "Shooter");
            intake = hardwareMap.get(DcMotor.class, "Intake");
            left_Transfer = hardwareMap.get(CRServo.class, "Left Transfer");
            right_Transfer = hardwareMap.get(CRServo.class, "Right Transfer");

            telemetry.addData("Hardware", "Initialized Successfully");
        } catch (Exception e) {
            telemetry.addData("Hardware Error", e.getMessage());
        }

        telemetry.addData("Status", "Blue Wall Auto Initialized");
        telemetry.addData("Start", "X: %.1f, Y: %.1f, H: 270°",
                startPose.getX(), startPose.getY());
        telemetry.addData("Score Pose", "X: %.1f, Y: %.1f, H: 270°",
                scorePose.getX(), scorePose.getY());
        telemetry.addData("Shoot Pose", "X: %.1f, Y: %.1f, H: 325°",
                shootPose.getX(), shootPose.getY());
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        actionTimer.resetTimer();
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
        telemetry.addData("Action Timer", "%.1f sec", actionTimer.getElapsedTimeSeconds());
        telemetry.addData("Shooter Power", shooter.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        // Make sure everything stops
        shooter.setPower(0);
        intake.setPower(0);
        left_Transfer.setPower(0);
        right_Transfer.setPower(0);

        telemetry.addData("Status", "Blue Wall Auto Complete");
        telemetry.addData("Final Time", "%.2f seconds", opmodeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
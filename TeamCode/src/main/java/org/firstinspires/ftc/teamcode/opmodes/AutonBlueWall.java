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

@Autonomous(name = "Blue Wall Auto", group = "Autonomous")
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
    private final Pose scorePose = new Pose(72.000, 77.000, Math.toRadians(270));
    private final Pose shootPose = new Pose(72.000, 77.000, Math.toRadians(325));

    //private final Pose scanPose = new Pose(71.500, 119.000, Math.toRadians(270));
    //private final Pose prepToPickup = new Pose(40.750, 84.250, Math.toRadians(180));
    //private final Pose pickup1Pose = new Pose(10.000, 84.000, Math.toRadians(180));
    //private final Pose scorePickup1Pose = new Pose(53.200, 106.200, Math.toRadians(315));
    //private final Pose parkPose = new Pose(40.500, 60.200, Math.toRadians(180));

    private PathChain path1, path2; //, path3, path4, path5, path6;

    public void buildPaths() {
        // Path 1: Start to Score Position (no rotation yet)
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        // Path 2: Rotate in place from 270° to 325° (counter-clockwise)
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, shootPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), shootPose.getHeading())
                .build();

        // Path 2: Score to Scan Position
        //path2 = follower.pathBuilder()
        //        .addPath(new BezierLine(scorePose, scanPose))
        //        .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(270))
        //        .build();

        // Path 3: Scan to Prep for Pickup
        //path3 = follower.pathBuilder()
        //        .addPath(new BezierLine(scanPose, prepToPickup))
        //        .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
        //        .build();

        // Path 4: Prep to Pickup Sample
        //path4 = follower.pathBuilder()
        //        .addPath(new BezierLine(prepToPickup, pickup1Pose))
        //        .setTangentHeadingInterpolation()
        //        .build();

        // Path 5: Pickup to Score Sample
        //path5 = follower.pathBuilder()
        //        .addPath(new BezierLine(pickup1Pose, scorePickup1Pose))
        //        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
        //        .build();

        // Path 6: Score to Park
        //path6 = follower.pathBuilder()
        //        .addPath(new BezierLine(scorePickup1Pose, parkPose))
        //        .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
        //        .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start: Move to position (72, 57) maintaining 270° heading
                follower.followPath(path1, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    // Robot is at (72, 57) facing 270°
                    // Now rotate counter-clockwise to 325°
                    follower.followPath(path2, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    // Robot is now facing 325°
                    // Start shooting sequence
                    shooter.setDirection(DcMotorSimple.Direction.FORWARD);
                    shooter.setPower(0.8);
                    actionTimer.resetTimer();
                    setPathState(3);
                }
                break;

            case 3:
                // Wait for shooter to spin up (0.5 seconds)
                if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                    // Run transfer to feed specimen into shooter
                    left_Transfer.setPower(-1);
                    right_Transfer.setPower(1);
                    setPathState(4);
                }
                break;

            case 4:
                // Wait for transfer to complete (1.5 seconds total)
                if (actionTimer.getElapsedTimeSeconds() > 1.5) {
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
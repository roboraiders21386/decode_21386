package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BLUE GOAL AUTO", group = "Autonomous")
public class AutonBlueGoal extends OpMode {
    //private Limelight3A cam;

    private Follower follower;
    private Timer pathTimer, opmodeTimer, actionTimer;

    public enum PATH_STATE{
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD
    }
    private int pathState;

    private DcMotor shooter;
    private DcMotor intake;
    private CRServo left_Transfer;
    private CRServo right_Transfer;

    // shooter velocity target (ticks/sec)
    double targetVelocity = 1500;
    double increment = 75;

    final double NOMINAL_VOLTAGE = 12.0;
    double compensated;

//    private final Pose startPose = new Pose(24.000, 127.000, Math.toRadians(317));
//    private final Pose scorePose = new Pose(55.000, 87.000, Math.toRadians(307));
//    private final Pose intakePose = new Pose(48.000, 60.000, Math.toRadians(90));

    private final Pose startPose = new Pose(24, 127, Math.toRadians(315)); // Start Pose of our robot.
    // Initialize poses
    private final Pose PPGPose = new Pose(48, 83.5, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose PGPPose = new Pose(48, 59.5, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose GPPPose = new Pose(48, 35.5, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose scorePose = new Pose(60, 85, Math.toRadians(315)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

//    private PathChain path1, path2;
//
//    public void buildPaths() {
//        path1 = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, scorePose))
//                .setConstantHeadingInterpolation(startPose.getHeading())
//                .build();
//
//        path2 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, intakePose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), intakePose.getHeading())
//                .build();
//    }

    //private Path scorePreload;
    private PathChain driveStartShoot, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        //scorePreload = new Path(new BezierLine(startPose, scorePose));
        //scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        driveStartShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, GPPPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), GPPPose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(GPPPose, scorePose))
                .setLinearHeadingInterpolation(GPPPose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, PGPPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), PGPPose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(PGPPose, scorePose))
                .setLinearHeadingInterpolation(PGPPose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, PPGPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), PPGPose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(PPGPose, scorePose))
                .setLinearHeadingInterpolation(PPGPose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(driveStartShoot, true);
                shooter.setPower(0.9);
                setPathState(20);
                break;
            case 20:
                if(!follower.isBusy()&& pathTimer.getElapsedTimeSeconds()>3) {
                    intake.setPower(1);
                    left_Transfer.setPower(1);
                    right_Transfer.setPower(-1);
                    setPathState(1);
                }
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy()) {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if((!follower.isBusy()) && pathTimer.getElapsedTimeSeconds() > 7) {
                    shooter.setPower(0);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1,true);
                    telemetry.addLine("Done grabPickup1 path");
                    setPathState(2);

                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1,true);
                    telemetry.addLine("Done scorePickup1 path");
                    setPathState(16);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2,true);
                    telemetry.addLine("Done grabPickup2 path");

                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2,true);
                    telemetry.addLine("Done scorePickup2 path");
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup3,true);
                    telemetry.addLine("Done grabPickup3 path");

                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    telemetry.addLine("Done scorePickup3 path");

                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
            default:
                telemetry.addLine("No valid state");
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                follower.followPath(path1, true);
//                setPathState(1);
//                break;
//
//            case 1:
//                if (!follower.isBusy()) {
//                    actionTimer.resetTimer();
//                    setPathState(-1);
//                    //setPathState(2);
//                }
//                break;
//
//            case 2:
//                if (actionTimer.getElapsedTimeSeconds() > 0.1) {
//                    shooter.setPower(0.9);
//                }
//                if (actionTimer.getElapsedTimeSeconds() > 1.5) {
//                    left_Transfer.setPower(-1);
//                    right_Transfer.setPower(1);
//                    intake.setPower(1);
//                    setPathState(3);
//                }
//                break;
//
//            case 3:
//                if (actionTimer.getElapsedTimeSeconds() > 5.5) {
//                    left_Transfer.setPower(0);
//                    right_Transfer.setPower(0);
//                    shooter.setPower(0);
//                    intake.setPower(0);
//
//                    follower.followPath(path2, true);
//                    setPathState(4);
//                }
//                break;
//
//            case 4:
//                if (!follower.isBusy()) {
//                    setPathState(-1);
//                }
//                break;
//
//        }
//    }



    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathState = 0;
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();
        //cam = hardwareMap.get(Limelight3A.class, "limelight");

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose); //set the starting pose

        try {
            shooter = hardwareMap.get(DcMotor.class, "Shooter");
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intake = hardwareMap.get(DcMotor.class, "Intake");
            left_Transfer = hardwareMap.get(CRServo.class, "Left Transfer");
            right_Transfer = hardwareMap.get(CRServo.class, "Right Transfer");
            telemetry.addData("Hardware", "Initialized Successfully");
        } catch (Exception e) {
            telemetry.addData("Hardware Error", e.getMessage());
        }

        telemetry.addData("Status", "Blue Goal Auto Initialized");
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.update();

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(pathState);
        actionTimer.resetTimer();
        shooter.setPower(0);
        intake.setPower(0);
        left_Transfer.setPower(0);
        right_Transfer.setPower(0);
       // cam.start();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        if (opmodeTimer.getElapsedTimeSeconds() > 10) {
            requestOpModeStop();
        }
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Time", "%.1f sec", opmodeTimer.getElapsedTimeSeconds());
        telemetry.addData("Action Timer", "%.1f sec", actionTimer.getElapsedTimeSeconds());
        telemetry.addData("Shooter Power", shooter.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.setPower(0);
        intake.setPower(0);
        left_Transfer.setPower(0);
        right_Transfer.setPower(0);

        telemetry.addData("Status", "Blue Goal Auto Complete");
        telemetry.addData("Final Time", "%.2f seconds", opmodeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
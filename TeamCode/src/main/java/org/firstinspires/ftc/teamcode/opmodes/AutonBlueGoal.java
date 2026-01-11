package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.pedropathing.ftc.FTCCoordinates;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BLUE SHORT RANGE AUTO", group = "Autonomous")
public class AutonBlueGoal extends OpMode {
    //private Limelight3A cam;

    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    public enum PATH_STATE{
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD
    }
    private int pathState;

    private DcMotorEx shooter;
    private DcMotor intake;
    private CRServo left_Transfer;
    private CRServo right_Transfer;

    // ==================== CONSTANTS - TIMING ====================
    private static final double shooterLoading = 2; // seconds for shooter to reach full speed this is essential
    private static final double shooterDuration = 4; // seconds to shoot 3 artifacts
    private static final double intakeDuration = 1; // seconds to intake a sample
    private static final double auto = 30; // total autonomous time limit

    // shooter velocity target (ticks/sec)
    double targetVelocity = 1150;
    double increment = 75;

    final double NOMINAL_VOLTAGE = 12.0;
    double compensated;
    private VoltageSensor voltageSensor;

    // Shooter velocity targets
    private double kP = 0.0001; // Proportional gain for velocity control (tune this!)

    double sp = 0.6;

//    private final Pose startPose = new Pose(24.000, 127.000, Math.toRadians(317));
//    private final Pose scorePose = new Pose(55.000, 87.000, Math.toRadians(307));
//    private final Pose intakePose = new Pose(48.000, 60.000, Math.toRadians(90));

    private final Pose startPose = new Pose(24, 134, Math.toRadians(315)); // Start Pose of our robot.
    // Initialize poses
    private final Pose PPGPose = new Pose(55, 85, Math.toRadians(175)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose PPGcollected = new Pose(20,85,Math.toRadians(175));
    private final Pose PGPcollected = new Pose(20,53,Math.toRadians(190));

    private final Pose PGPPose = new Pose(55, 58, Math.toRadians(160)); // Middle (Second Set) of Artifacts from the Spike Mark.

    private final Pose scorePose = new Pose(70, 87, Math.toRadians(315)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePose2 = new Pose(65, 87, Math.toRadians(320)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePose3 = new Pose(65, 87, Math.toRadians(315)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.


    private final Pose endPose = new Pose(48, 60, Math.toRadians(160)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.


    private PathChain goToShootPreload, goToIntake, collectArtifacts,goToShoot1, goToIntake1, collectArtifacts1, goToShoot2, endAuto;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        //scorePreload = new Path(new BezierLine(startPose, scorePose));
        //scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        goToShootPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        goToIntake = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, PPGPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), PPGPose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        collectArtifacts = follower.pathBuilder()
                .addPath(new BezierLine(PPGPose, PPGcollected))
                .setLinearHeadingInterpolation(PPGPose.getHeading(), PPGcollected.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        goToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(PPGcollected, scorePose2))
                .setLinearHeadingInterpolation(PPGcollected.getHeading(), scorePose2.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        goToIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, PGPPose))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), PGPPose.getHeading())
                .build();


        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        collectArtifacts1 = follower.pathBuilder()
                .addPath(new BezierLine(PGPPose, PGPcollected))
                .setLinearHeadingInterpolation(PGPPose.getHeading(), PGPcollected.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        goToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(PGPcollected, scorePose3))
                .setLinearHeadingInterpolation(PGPcollected.getHeading(), scorePose3.getHeading())
                .build();
        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        endAuto = follower.pathBuilder()
                .addPath(new BezierLine(scorePose3, endPose))
                .setLinearHeadingInterpolation(scorePose3.getHeading(), endPose.getHeading())
                .build();
    }

  /*  private double getVoltageCompensatedVelocity() {
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double voltageComp = NOMINAL_VOLTAGE / voltage;
        return targetVelocity * voltageComp;
    }

   */

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(goToShootPreload, 0.5,true);
                setPathState(34);
                break;

            case 34:
                if(!follower.isBusy()) {
                    shooter.setDirection(DcMotorSimple.Direction.FORWARD);
                    shooter.setVelocity(targetVelocity-100);
                    setPathState(20);
                }
                break;
            case 20:
                if(!follower.isBusy()&& pathTimer.getElapsedTimeSeconds()>shooterLoading) {
                    intake.setPower(1);
                    left_Transfer.setPower(1);
                    right_Transfer.setPower(-1);
                    pathTimer.resetTimer();
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
                if((!follower.isBusy()) && pathTimer.getElapsedTimeSeconds()>shooterDuration) {
                    left_Transfer.setPower(0);
                    right_Transfer.setPower(0);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(goToIntake,0.5,true);
                    telemetry.addLine("Done grabPickup1 path");
                    pathTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>intakeDuration) {
                    /* Grab artifacts */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(collectArtifacts,0.5,true);
                    telemetry.addLine("Done collecting artifacts");
                    pathTimer.resetTimer();
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>4) {
                    /* Score artifact */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(goToShoot1,true);
                    shooter.setVelocity(targetVelocity);
                    telemetry.addLine("Done shooting pickups");
                    pathTimer.resetTimer();
                    setPathState(492);
                }
                break;
            case 492:
                if(!follower.isBusy()){
                    intake.setPower(1);
                    left_Transfer.setPower(1);
                    right_Transfer.setPower(-1);
                    pathTimer.resetTimer();
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>shooterDuration) {
                    /* Grab Sample */
                    left_Transfer.setPower(0);
                    right_Transfer.setPower(0);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(goToIntake1,true);
                    telemetry.addLine("Pickup some more");
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(collectArtifacts1,0.5,true);
                    telemetry.addLine("Done grabPickup3 path");

                    setPathState(94);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>4) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(goToShoot2, true);
                    telemetry.addLine("Done scorePickup3 path");

                    setPathState(40);
                }
                break;
            case 40:
                if(pathTimer.getElapsedTimeSeconds()>intakeDuration){
                    intake.setPower(1);
                    left_Transfer.setPower(1);
                    right_Transfer.setPower(-1);
                    setPathState(94);
                    pathTimer.resetTimer();
                }
                break;
            case 94:
                if(pathTimer.getElapsedTimeSeconds()>shooterDuration){
                    follower.followPath(endAuto,true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    //end auto
                    stop();
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




    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathState = 0;
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        pathTimer.resetTimer();
        opmodeTimer.resetTimer();
        //cam = hardwareMap.get(Limelight3A.class, "limelight");

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose); //set the starting pose
        follower.setMaxPower(0.5);

        try {
            shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intake = hardwareMap.get(DcMotor.class, "Intake");
            left_Transfer = hardwareMap.get(CRServo.class, "Left Transfer");
            right_Transfer = hardwareMap.get(CRServo.class, "Right Transfer");
            telemetry.addData("Hardware", "Initialized Successfully");
            //battery compensation
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
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
    public void init_loop() {
        follower.update();

        telemetry.addData("Status", "Ready");
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Expected", "315°");
        telemetry.update();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(pathState);
        pathTimer.resetTimer();
        shooter.setPower(0);
        intake.setPower(0);
        left_Transfer.setPower(0);
        right_Transfer.setPower(0);
        // cam.start();
    }
    private double calculateShooterPower(double targetVelocity) {
        // Get current velocity in ticks per second
        double currentVelocity = shooter.getVelocity();

        // Calculate voltage compensation
        double currentVoltage = voltageSensor.getVoltage();
        double voltageCompensation = NOMINAL_VOLTAGE / currentVoltage;

        // Simple proportional control
        double error = targetVelocity - currentVelocity;
        double basePower = (targetVelocity / 2800.0); // Assuming max ~2800 ticks/sec at full power
        double correction = error * kP;

        // Apply voltage compensation and correction
        double power = (basePower + correction) * voltageCompensation;

        // Clamp power between 0 and 1
        return Math.max(0, Math.min(1.0, power));
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

       /* if (opmodeTimer.getElapsedTimeSeconds() > auto) {
            stop();
            requestOpModeStop();
        }

        */
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Time", "%.1f sec", opmodeTimer.getElapsedTimeSeconds());
        telemetry.addData("Path Timer", "%.1f sec", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Shooter Power", shooter.getPower());
        ShooterSpeed();
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
    private void ShooterSpeed() {
        double currentVelocity = shooter.getVelocity();
        //double targetVelocity = getVoltageCompensatedVelocity();
        telemetry.addData("Shooter Speed: ", (Math.abs(currentVelocity - targetVelocity)));
        telemetry.update();
    }
}
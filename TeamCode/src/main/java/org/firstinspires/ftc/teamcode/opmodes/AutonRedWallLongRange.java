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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Autonomous(name = "RED WALL LONG RANGE", group = "Autonomous")
public class AutonRedWallLongRange extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private DcMotorEx shooter;
    private DcMotor intake;
    private CRServo left_Transfer;
    private CRServo right_Transfer;
    private Servo hood;
    // Shooter settings
    private final double LONG_RANGE_VELOCITY = 1700;
    private final double NOMINAL_VOLTAGE = 12.0;
    // Paths
    private final Pose startPose = new Pose(87.652, 8, Math.toRadians(270)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(84, 24, Math.toRadians(245)); // Score Pose of our robot.
    private Path goToShootPreload;
    private PathChain goToIntake, collectArtifacts, goToShoot1, goToPickupHP, collectArtifactsHP, goToShootHP,endAuto;
    public void buildPaths() {

        goToShootPreload = new Path(new BezierLine(startPose, scorePose));
        goToShootPreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
//        goToShootPreload = follower.pathBuilder()
//                .addPath(new BezierLine(new Pose(87.652, 8), new Pose(84, 24)))
//                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(245))
//                .build();
        goToIntake = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(84, 24), new Pose(102.823, 40)))
                .setLinearHeadingInterpolation(Math.toRadians(245), Math.toRadians(0))
                .build();
        collectArtifacts = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(102.823, 40), new Pose(144, 40)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        goToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(137, 40), new Pose(85.5, 24)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(245))
                .build();
        goToPickupHP = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(85.5, 24), new Pose(130, 40)))
                .setLinearHeadingInterpolation(Math.toRadians(245), Math.toRadians(315))
                .build();
        collectArtifactsHP = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(130,40 ), new Pose(140, 5)))
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(315))
                .build();
        goToShootHP = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(140, 5), new Pose(84, 24)))
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(0))
                .build();
    }
    private double getVoltageCompensatedVelocity() {
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double voltageComp = NOMINAL_VOLTAGE / voltage;
        return LONG_RANGE_VELOCITY * voltageComp;
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Path 1: Drive to first shooting position
                follower.followPath(goToShootPreload, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    // At shooting position - spin up shooter
                    shooter.setDirection(DcMotorSimple.Direction.FORWARD);
                    shooter.setVelocity(getVoltageCompensatedVelocity());
                    hood.setPosition(0.1);
                    pathTimer.resetTimer();
                    setPathState(2);
                }
                break;

            case 2:
                // Wait 3 seconds for shooter to reach velocity
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    // Start transfer + intake to shoot
                    intake.setPower(1);
                    left_Transfer.setPower(1);
                    right_Transfer.setPower(-1);
                    pathTimer.resetTimer();
                    setPathState(563);
                }
                break;

            case 563:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>4) {
                    left_Transfer.setPower(0);
                    right_Transfer.setPower(0);
                    // Path 2: Go to intake position
                    follower.followPath(goToIntake, true);
                    pathTimer.resetTimer();
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()&& pathTimer.getElapsedTimeSeconds()>2) {
                    // Start intake
                    // Path 3: Drive into balls while intaking
                    follower.followPath(collectArtifacts,0.5,true);
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>5) {
                    // Path 4: Return to shooting position
                    follower.followPath(goToShoot1, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    // At shooting position - spin up shooter again
                    shooter.setDirection(DcMotorSimple.Direction.FORWARD);
                    shooter.setVelocity(getVoltageCompensatedVelocity());
                    pathTimer.resetTimer();
                    setPathState(7);
                }
                break;

            case 7:
                // Wait 3 seconds for shooter
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    // Start transfer to shoot
                    intake.setPower(1);
                    left_Transfer.setPower(1);
                    right_Transfer.setPower(-1);
                    pathTimer.resetTimer();
                    setPathState(8);
                }
                break;

            case 8:
                // Shoot for 5 seconds
                if (pathTimer.getElapsedTimeSeconds() > 5.0) {
                    // Stop shooting
                    left_Transfer.setPower(0);
                    right_Transfer.setPower(0);
                    shooter.setVelocity(0);

                    // Path 5: Go to second intake position
                    follower.followPath(goToPickupHP, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    // Start intake for second pickup
                    intake.setDirection(DcMotorSimple.Direction.FORWARD);
                    intake.setPower(1);
                    // Path 6: Drive to pick up balls
                    follower.followPath(collectArtifactsHP,0.5, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    // Stop intake after collecting
                    intake.setPower(0);

                    // Path 7: Go to final shooting position
                    follower.followPath(goToShootHP, true);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    // At final shooting position - spin up shooter
                    shooter.setDirection(DcMotorSimple.Direction.FORWARD);
                    shooter.setVelocity(getVoltageCompensatedVelocity());
                    pathTimer.resetTimer();
                    setPathState(12);
                }
                break;

            case 12:
                // Wait 3 seconds for shooter
                if (pathTimer.getElapsedTimeSeconds() > 3.0) {
                    // Start transfer to shoot
                    left_Transfer.setPower(1);
                    right_Transfer.setPower(-1);
                    setPathState(13);
                }
                break;

            case 13:
                // Final shooting for 4 seconds
                if (pathTimer.getElapsedTimeSeconds() > 4.0) {
                    // Stop everything
                    left_Transfer.setPower(0);
                    right_Transfer.setPower(0);
                    shooter.setVelocity(0);
                    intake.setPower(0);

                    // Autonomous complete
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()){
                    follower.followPath(endAuto, true);
                    setPathState(-1);
                }
                break;

            default:
                // Ensure everything is stopped
                shooter.setVelocity(0);
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
        opmodeTimer.resetTimer();
        // Initialize Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(new Pose(87.652, 8.187, Math.toRadians(270)));
        follower.setMaxPower(0.75);
        pathState = 0;
        // Initialize hardware
        try {
            shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter.setDirection(DcMotorSimple.Direction.FORWARD);
            intake = hardwareMap.get(DcMotor.class, "Intake");
            left_Transfer = hardwareMap.get(CRServo.class, "Left Transfer");
            right_Transfer = hardwareMap.get(CRServo.class, "Right Transfer");
            hood = hardwareMap.get(Servo.class,"hood");
            telemetry.addData("Hardware", "Initialized Successfully");
        } catch (Exception e) {
            telemetry.addData("Hardware Error", e.getMessage());
        }
        telemetry.addData("Status", "Red Wall Long Range Auto Initialized");
        telemetry.addData("Mode", "LONG RANGE");
        telemetry.addData("Target Velocity", LONG_RANGE_VELOCITY);
        telemetry.update();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        follower.update();

        telemetry.addData("Status", "Ready");
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Expected", "270°");
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
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        telemetry.addData("Path State", pathState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Position", "X: %.1f, Y: %.1f",
                follower.getPose().getX(), follower.getPose().getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Battery Voltage", "%.2f V", voltage);
        telemetry.addData("Target Velocity", LONG_RANGE_VELOCITY);
        telemetry.addData("Actual Velocity", shooter.getVelocity());
        telemetry.addData("Time", "%.1f sec", opmodeTimer.getElapsedTimeSeconds());
        telemetry.addData("Path Timer", "%.1f sec", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
    @Override
    public void stop() {
        shooter.setVelocity(0);
        intake.setPower(0);
        left_Transfer.setPower(0);
        right_Transfer.setPower(0);
        telemetry.addData("Status", "Red Wall Long Range Auto Complete");
        telemetry.addData("Final Time", "%.2f seconds", opmodeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
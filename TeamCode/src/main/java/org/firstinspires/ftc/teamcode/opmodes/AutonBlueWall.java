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

    private DcMotor shooter;
    private DcMotor intake;
    private CRServo left_Transfer;
    private CRServo right_Transfer;

    private final Pose startPose = new Pose(84.000, 8.000, Math.toRadians(270));
    private final Pose scorePose = new Pose(60.000, 74.000, Math.toRadians(315));
    private final Pose intakePose = new Pose(48.000, 60.000, Math.toRadians(90));

    private PathChain path1, path2;

    public void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intakePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intakePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    shooter.setDirection(DcMotorSimple.Direction.FORWARD);
                    actionTimer.resetTimer();
                    setPathState(2);
                }
                break;

            case 2:
                // Spin up shooter after brief delay
                if (actionTimer.getElapsedTimeSeconds() > 0.1) {
                    shooter.setPower(0.9);
                }
                // After 1.5 seconds, start transfer AND intake
                if (actionTimer.getElapsedTimeSeconds() > 1.5) {
                    left_Transfer.setPower(-1);
                    right_Transfer.setPower(1);
                    intake.setDirection(DcMotorSimple.Direction.FORWARD);
                    intake.setPower(1);
                    setPathState(3);
                }
                break;

            case 3:
                // Run all three for 4 more seconds (5.5 total)
                if (actionTimer.getElapsedTimeSeconds() > 5.5) {
                    left_Transfer.setPower(0);
                    right_Transfer.setPower(0);
                    shooter.setPower(0);
                    intake.setPower(0);

                    follower.followPath(path2, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

            default:
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

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        pathState = 0;

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
        telemetry.addData("Start", "X: %.1f, Y: %.1f, H: 0°",
                startPose.getX(), startPose.getY());
        telemetry.addData("Score Pose", "X: %.1f, Y: %.1f, H: 315°",
                scorePose.getX(), scorePose.getY());
        telemetry.addData("Intake Pose", "X: %.1f, Y: %.1f, H: 90°",
                intakePose.getX(), intakePose.getY());
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
        shooter.setPower(0);
        intake.setPower(0);
        left_Transfer.setPower(0);
        right_Transfer.setPower(0);

        telemetry.addData("Status", "Blue Wall Auto Complete");
        telemetry.addData("Final Time", "%.2f seconds", opmodeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
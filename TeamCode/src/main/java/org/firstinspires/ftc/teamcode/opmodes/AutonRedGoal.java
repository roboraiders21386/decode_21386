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

@Autonomous(name = "RUN RED GOAL", group = "Autonomous")
public class AutonRedGoal extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer, actionTimer;
    private int pathState;

    // Hardware
    private DcMotor shooter;
    private DcMotor intake;
    private CRServo left_Transfer;
    private CRServo right_Transfer;

    // Red Goal Starting Position and Poses (mirrored from Blue Goal)
    private final Pose startPose = new Pose(120.000, 127.000, Math.toRadians(223));
    private final Pose scorePose = new Pose(89.000, 87.000, Math.toRadians(233));
    private final Pose intakePose = new Pose(96.000, 60.000, Math.toRadians(90));

    private PathChain path1, path2;

    public void buildPaths() {
        // Path 1: Start (120, 127) to Score Position (89, 87) with heading 233°
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        // Path 2: Score position to intake position (96, 60) at 90° heading
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intakePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intakePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start: Move to scoring position (89, 87) at 233° heading
                follower.followPath(path1, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    // Robot is at scoring position (89, 87) facing 233°
                    // Start shooter spin-up
                    shooter.setDirection(DcMotorSimple.Direction.FORWARD);
                    shooter.setPower(0.9);
                    actionTimer.resetTimer();
                    setPathState(2);
                }
                break;

            case 2:
                // Wait 3 seconds for shooter to spin up
                if (actionTimer.getElapsedTimeSeconds() > 3.0) {
                    // Now start transfer servos while keeping shooter running
                    left_Transfer.setPower(-1);
                    right_Transfer.setPower(1);
                    setPathState(3);
                }
                break;

            case 3:
                // Run both shooter and transfer for 4 more seconds (7 total)
                if (actionTimer.getElapsedTimeSeconds() > 7.0) {
                    // Stop transfer and shooter
                    left_Transfer.setPower(0);
                    right_Transfer.setPower(0);
                    shooter.setPower(0);

                    // Now move to intake position
                    follower.followPath(path2, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    // Robot is at intake position (96, 60) facing 90°
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

        telemetry.addData("Status", "Red Goal Auto Initialized");
        telemetry.addData("Start", "X: %.1f, Y: %.1f, H: 223°",
                startPose.getX(), startPose.getY());
        telemetry.addData("Score Pose", "X: %.1f, Y: %.1f, H: 233°",
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
        // Make sure everything stops
        shooter.setPower(0);
        intake.setPower(0);
        left_Transfer.setPower(0);
        right_Transfer.setPower(0);

        telemetry.addData("Status", "Red Goal Auto Complete");
        telemetry.addData("Final Time", "%.2f seconds", opmodeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
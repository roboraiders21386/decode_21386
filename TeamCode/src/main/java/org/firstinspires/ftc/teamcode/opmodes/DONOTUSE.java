package org.firstinspires.ftc.teamcode.opmodes; // make sure this aligns with class location
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous(name = "DO NOT USE", group = "Autonomous")
public class DONOTUSE extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private DcMotor intake;
    private DcMotor shooter;
    private CRServo lTransfer;
    private CRServo rTransfer;

    private final Pose startPose = new Pose(21, 130, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(72, 72, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scanPose = new Pose(72, 120, Math.toRadians(45)); // scanning Pose
    private final Pose prepToPickup = new Pose(41
            , 84, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1Pose = new Pose(11, 84, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private Path scorePreload;
    private PathChain gotoGrabPickup1, scanCode, scorePickup1, grabPickup1, scorePickup2, grabPickup3, scorePickup3;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = hardwareMap.get(DcMotor.class, "Shooter");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        lTransfer = hardwareMap.get(CRServo.class, "Left Transfer");
        rTransfer = hardwareMap.get(CRServo.class, "Right Transfer");

        //  Initialize follower & paths
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        //  Initialize timers
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        waitForStart();

        if(opModeIsActive() && !isStopRequested()){
            autonomousPathUpdate();
            follower.update(); // keep updating follower in loop
            telemetry.addData("Path State", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }

    }
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line.*/
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our gotoGrabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scanCode = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, scanPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), scanPose.getHeading())
                .build();

        /* This is our gotoGrabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        gotoGrabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, prepToPickup))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prepToPickup.getHeading())
                .build();

        /* This is our grabPickUp1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(prepToPickup, pickup1Pose))
                .setLinearHeadingInterpolation(prepToPickup.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line.*/
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                safeWaitSeconds(0);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scanCode, true);
                    safeWaitSeconds(10);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(gotoGrabPickup1, true);
                    safeWaitSeconds(5);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1, true);
                    safeWaitSeconds(5);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1, true);
                    safeWaitSeconds(5);
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }




    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.seconds() < time) {
            follower.update(); // keep follower running during wait
        }
    }


}



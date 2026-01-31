/** i randomly created ts at 1am its really bad helppp
 *  **Automatic Shooter Aiming System**
 * The robot continuously looks for AprilTags #20 or #24 (on the goal)
 * - Calculates distance to the basket using camera input and trigonometry
 * - **Automatically sets shooter velocity** based on distance (closer = slower, farther = faster)
 * - **Automatically adjusts hood angle** for proper trajectory
 * - **Switches PIDF coefficients** dynamically (short range vs long range tuning)
 * - just aim generally at the target!

 * ### 2. **Driver Controls**

 * **Gamepad 1 (Driver):**
 * - **Left Stick** - Strafe and forward/backward
 * - **Right Stick** - Rotate robot
 * - **Right Bumper** - Potential Slow mode (50% speed)

 * **Gamepad 2 (Operator):**
 * - **Left Trigger** - Run intake collecting artifacts
 * - **Circle** - Reverse intake (clear jams)
 * - **Triangle** - Run transfer system at shooting position
 * - **Square** - Stop transfer only
 * - **Cross** - Emergency stop everything
 * - **Right Trigger** - Spin up shooter (uses auto-calculated velocity)
 * - **Right Bumper** - Stop shooter

 * ### 3. **Smart Artifact Detection**
 * - **Distance sensor** detects when a artifact is in the intake
 * - After 2 seconds of detection, automatically stops intake

 * ### 4. **Shooter Feedback**
 * - **Controller rumbles** when shooter reaches target speed
 * - **Telemetry displays "READY TO SHOOT"** when aligned and at speed
 * - Shows distance, velocity, and hood position in real-time
 */

package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name = "velocity comp teleop test", group = "Testing")
@Disabled
public class randomthing extends OpMode {

    // hardware
    private Limelight3A camera;
    private Follower follower;
    private DcMotor RF, LF, RB, LB;
    private DcMotor intake;
    private DcMotorEx shooter;
    private CRServo right_Transfer, left_Transfer;
    private Servo hood;
    private DistanceSensor colorDistance;

    // limelight constants
    // TODO: TUNE THESE VALUES FOR YOUR ROBOT
    private static final double camHeight = 10.0;  // Height of camera from ground
    private static final double targetHeight = 36;  // Height of basket/target
    private static final double camAngle = 15.0;  // Camera mount angle (upward positive)
    private double kP = 0.015;               // smaller for smoother rotation
    private double tolerance = 2.5;          // slightly larger deadzone
    private double maxRotationPower = 0.5;

    private boolean autoAimActive = false;


    // Format: {distance in inches, hood servo position 0.0-1.0}
    //hood change by distance
    private static final double[][] distancevshood = {
            //regression y=-0.000803571x+0.211786
            {96, 0.2},
            {84, 0.18},
            {72, 0.17},
            {60, 0.16},
            {48, 0.15},
            {36, 0.145},
            {24, 0.14}
    };
//end of testable values

    // pidf coeffs
    public static final double SHORT_P = 900, SHORT_F = 17.5, LONG_P = 900, LONG_F = 17.5;
    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double LONG_RANGE_THRESHOLD = 120.0; // inches
    // Preset shooter velocities (ticks/sec)
    final double LONG_RANGE_VELOCITY  = 1800;
    final double SHORT_RANGE_VELOCITY = 1200;



    // ===== STATE VARIABLES =====
    private double targetVelocity = 0;
    private double targetDistance = 0;
    private String shooterMode = "OFF";
    private boolean targetDetected = false;

    // Distance sensor for sample detection
    private ElapsedTime detectionTimer = new ElapsedTime();
    private boolean isArtifactPresent = false;
    private static final double DETECTION_TIME = 2.0;

    // Pedro Pathing
    private boolean following = false;
    public Pose startingPose = new Pose(0,0,0);

    private DcMotorControllerEx motorControllerEx;
    private int motorIndex;
    private PIDFCoefficients pidfShort, pidfLong;

    @Override
    public void init() {
        // drive init
        RF = hardwareMap.get(DcMotor.class, "RF");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        LB = hardwareMap.get(DcMotor.class, "LB");

        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.FORWARD);
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);

        // subsystems init
        intake = hardwareMap.get(DcMotor.class, "Intake");
        left_Transfer = hardwareMap.get(CRServo.class, "Left Transfer");
        right_Transfer = hardwareMap.get(CRServo.class, "Right Transfer");
        hood = hardwareMap.get(Servo.class, "hood");
        colorDistance = hardwareMap.get(DistanceSensor.class, "leftColorDistanceSensor");

        // shooter init
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        //pidf init
        motorControllerEx = (DcMotorControllerEx) shooter.getController();
        motorIndex = shooter.getPortNumber();


        // camera and pathing
        camera = hardwareMap.get(Limelight3A.class, "limelight");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);

        telemetry.addLine("System Ready");
        telemetry.addLine("Auto-aim activates when target detected");
        telemetry.update();
    }

    @Override
    public void start() {
        camera.start();
        camera.pipelineSwitch(0); // Set to your AprilTag/target detection pipeline
    }

    @Override
    public void loop() {
        follower.update();

        //automatic limelight processing
        processLimelightAutoAim();

        //gamepad controls
        Control();

        // telemetry
        displayTelemetry();
    }

    /**
     * This function should do TWO THINGS AT ONCE:
     * 1. Auto-Aim the Shooter (Hood Angle)
     * When it detects AprilTag 20 or 24:

     * Calculates distance to the target using the vertical angle (ty)
     * Sets shooter velocity based on that distance
     * Sets hood angle based on that distance
     * Switches PIDF coefficients (short vs long range)

     * 2. Auto-Rotate the Robot (Returns Rotation Power)
     * Returns a rotation correction value that:

     * Uses the horizontal angle (tx) to see if robot is aligned
     * Applies proportional control (P-controller) to turn toward target
     * Returns power value you add to your drive's rotation (rx)
     */

    private void processLimelightAutoAim() {
        LLResult result = camera.getLatestResult();

        if (result == null || !result.isValid()) {
            // No valid result - no rotation correction
            targetDetected = false;
            if (!shooterMode.equals("OFF")) {
                shooterMode = "NO TARGET (LAST: " + String.format("%.0f\"", targetDistance) + ")";
            }
            return;
        }

        List<FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            // No tags detected
            targetDetected = false;
            if (!shooterMode.equals("OFF")) {
                shooterMode = "NO TARGET (LAST: " + String.format("%.0f\"", targetDistance) + ")";
            }
            return;
        }

        // Find target tag
        //blue goal is target 20 red is 24
        //obelisk tag is 21,22, or 23 DONT DETECT THAT
        FiducialResult targetTag = null;
        for (FiducialResult tag : tags) {
            if (tag.getFiducialId() == 24) {
                targetTag = tag;
                break;
            }
        }

        if (targetTag == null) {
            // Target tags not found
            targetDetected = false;
            if (!shooterMode.equals("OFF")) {
                shooterMode = "NO TARGET (LAST: " + String.format("%.0f\"", targetDistance) + ")";
            }
            return;
        }

        // ===== TARGET FOUND - PROCESS AUTO-AIM =====
        targetDetected = true;

        // Get vertical offset (ty) from Limelight for distance calculation
        double ty = result.getTy();

        // Calculate distance using trigonometry
        targetDistance = calculateDistance(ty);

        // Interpolate shooter velocity and hood position based on distance
        double hoodPosition = interpolateHood(targetDistance);

        // Automatically apply settings
        hood.setPosition(hoodPosition);

        // Dynamically select PIDF based on calculated distance
        if (targetDistance > LONG_RANGE_THRESHOLD) {
            motorControllerEx.setPIDFCoefficients(motorIndex,
                    DcMotor.RunMode.RUN_USING_ENCODER, pidfLong);
            shooterMode = String.format("AUTO %.0f\" LONG", targetDistance);
        } else {
            motorControllerEx.setPIDFCoefficients(motorIndex,
                    DcMotor.RunMode.RUN_USING_ENCODER, pidfShort);
            shooterMode = String.format("AUTO %.0f\" SHORT", targetDistance);
        }

        double error = result.getTx() - 2.3; // Apply 2.3° offset

        if (Math.abs(error) <= tolerance) {
            return; // Within tolerance - no correction needed
        }

        double output = kP * error;
        output = Math.max(-maxRotationPower, Math.min(output, maxRotationPower));

    }


    // calculate distance
    private double calculateDistance(double ty) {
        // Formula: distance = (h2 - h1) / tan(a1 + a2)
        // h2 = target height, h1 = camera height
        // a1 = camera mount angle, a2 = ty from limelight

        double angleToTargetRadians = Math.toRadians(camAngle + ty);
        double heightDifference = targetHeight - camHeight;

        double distance = heightDifference / Math.tan(angleToTargetRadians);

        // Clamp distance to reasonable range (prevent division errors)
        return Math.max(12, Math.min(120, distance));
    }

    // ===== INTERPOLATION FUNCTIONS =====


    private double interpolateHood(double distance) {
        return linearInterpolate(distancevshood, distance);
    }

    private double linearInterpolate(double[][] map, double distance) {
        // Clamp to map bounds
        if (distance <= map[0][0]) return map[0][1];
        if (distance >= map[map.length-1][0]) return map[map.length-1][1];

        // Find surrounding points and interpolate
        for (int i = 0; i < map.length - 1; i++) {
            if (distance >= map[i][0] && distance <= map[i+1][0]) {
                double x1 = map[i][0];
                double y1 = map[i][1];
                double x2 = map[i+1][0];
                double y2 = map[i+1][1];

                // Linear interpolation
                return y1 + (distance - x1) * (y2 - y1) / (x2 - x1);
            }
        }

        return map[0][1]; // Fallback
    }

    // gamepad Controls
    private void Control() {
        PIDFCoefficients pidfshort = new PIDFCoefficients(SHORT_P, 0, 0, SHORT_F);
        PIDFCoefficients pidflong = new PIDFCoefficients(LONG_P, 0, 0, LONG_F);

        // ----------- MECANUM DRIVE -----------
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        double deadzone = 0.05;
        if (Math.abs(y)  < deadzone) y  = 0;
        if (Math.abs(x)  < deadzone) x  = 0;
        if (Math.abs(rx) < deadzone) rx = 0;

        x *= 1.1;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeft  = (y + x + rx) / denominator;
        double backLeft   = (y - x + rx) / denominator;
        double frontRight = (y - x - rx) / denominator;
        double backRight  = (y + x - rx) / denominator;

        double speed = gamepad1.right_bumper ? 0.5 : 1.0;

        LF.setPower(frontLeft  * speed);
        LB.setPower(backLeft   * speed);
        RF.setPower(frontRight * speed);
        RB.setPower(backRight  * speed);

        // ----- SENSING------
        if (Math.abs(targetVelocity-shooter.getVelocity())<=50)
            gamepad2.rumble(1.0, 1.0, 1000);
        // ----------- INTAKE -----------
        if (gamepad2.left_trigger > 0.3) {
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setPower(1);
        } else if (gamepad2.left_trigger < 0.3 && !gamepad2.circle && !gamepad2.square) {
            intake.setPower(0);
        }


        // ----------- TRANSFER / CLEARING -----------
        if (gamepad2.triangle) {
            left_Transfer.setPower(1);
            right_Transfer.setPower(-1);

        } else if (gamepad2.cross) {
            left_Transfer.setPower(0);
            right_Transfer.setPower(0);
            shooter.setVelocity(0);
            intake.setPower(0);
            shooterMode = "OFF";

        } else if (gamepad2.circle) {   // full reverse clear
            left_Transfer.setPower(-1);
            right_Transfer.setPower(1);

            shooter.setDirection(DcMotorSimple.Direction.REVERSE);
            shooter.setVelocity(1200);

            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setPower(.6);

        } else if (gamepad2.square) {   //stop transfer
            left_Transfer.setPower(0);
            right_Transfer.setPower(0);
        }


        // ----------- SHOOTER MODES (TWO PRESETS) -----------

        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double voltageComp = NOMINAL_VOLTAGE / voltage;

        // Select mode
        if (gamepad2.dpad_up) {
            targetVelocity = LONG_RANGE_VELOCITY;
            shooterMode = "LONG RANGE";
            sleep(150);
            hood.setPosition(.14);
            motorControllerEx.setPIDFCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER, pidflong);
            PIDFCoefficients pidfModified = motorControllerEx.getPIDFCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER);

        }

        if (gamepad2.dpad_down) {
            targetVelocity = SHORT_RANGE_VELOCITY;
            shooterMode = "SHORT RANGE";
            sleep(150);
            hood.setPosition(.2);
            motorControllerEx.setPIDFCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER, pidfshort);
            PIDFCoefficients pidfModified = motorControllerEx.getPIDFCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER);
        }

        double compensated = targetVelocity * voltageComp;


        // Run shooter when trigger is held
        if (gamepad2.right_trigger > 0.3) {
            shooter.setDirection(DcMotorSimple.Direction.FORWARD);
            shooter.setVelocity(compensated);

        } else if (gamepad2.right_bumper) {
            shooter.setVelocity(0);
            shooterMode = "OFF";
        }

            /*if(shooter.getVelocity() >= (0.8*targetVelocity)){
                led.enableLight(true);
                telemetry.addData("Is Led on? ", led.isLightOn());
            }*/

        double actualVelocity = shooter.getVelocity();


    }

    // telemetry to loop throughout
    private void displayTelemetry() {
        telemetry.addLine("========== AUTO-AIM STATUS ==========");
        telemetry.addData("Target Detected", targetDetected ? "✓ YES" : "✗ NO");
        telemetry.addData("Shooter Mode", shooterMode);

        if (targetDetected) {
            telemetry.addData("Distance", "%.1f inches", targetDistance);
            telemetry.addData("Auto Velocity", "%.0f tps", targetVelocity);
            telemetry.addData("Auto Hood", "%.3f", hood.getPosition());
        }

        telemetry.addLine();
        telemetry.addLine("========== SHOOTER STATUS ==========");
        telemetry.addData("Current Velocity", "%.0f tps", shooter.getVelocity());
        telemetry.addData("Error", "%.0f tps", targetVelocity - shooter.getVelocity());

        // Visual indicator for "ready to shoot"
        if (Math.abs(targetVelocity - shooter.getVelocity()) <= 50 && targetDetected) {
            telemetry.addLine(">>> READY TO SHOOT <<<");
        }

        telemetry.addLine();
        telemetry.addData("Battery", "%.1fV",
                hardwareMap.voltageSensor.iterator().next().getVoltage());
        telemetry.addData("Sample Distance", "%.1f cm",
                colorDistance.getDistance(DistanceUnit.CM));

        telemetry.update();
    }

    // other pedro stuff
    private Pose getRobotPoseFromCamera() {
        return new Pose(0, 0, 0, FTCCoordinates.INSTANCE)
                .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }

    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
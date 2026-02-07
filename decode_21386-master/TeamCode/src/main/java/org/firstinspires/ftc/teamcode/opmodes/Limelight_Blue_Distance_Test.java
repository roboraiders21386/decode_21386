package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name = "Limelight_Distance_Test")
public class Limelight_Blue_Distance_Test extends LinearOpMode {

    // Drive motors
    private DcMotor RF, LF, RB, LB;

    // Mechanisms
    private DcMotor intake;
    private DcMotorEx shooter;
    private CRServo right_Transfer;
    private CRServo left_Transfer;
    private Servo hood;
    private LED led;

    // Limelight and IMU
    private Limelight3A limelight;
    private IMU imu;
    private double orientation;

    // Auto-aim parameters
    private double minPower = 0.15;
    private double maxPower = 0.4;
    private double tolerance = .1;
    private double lastError = 0;
    private double rotationOffset = -1; // slight offset to aim slightly left

    // Shooter PIDF parameters
    private final double SHORT_P = 1000, SHORT_I = 0.001, SHORT_D = 0, SHORT_F = 18;
    private final double LONG_P  = 2500, LONG_I  = 0.001, LONG_D  = 0.001, LONG_F  = 18;

    // Shooter target
    double targetVelocity = 0;
    final double LONG_RANGE_VELOCITY  = 1700;
    final double SHORT_RANGE_VELOCITY = 1275;
    final double NOMINAL_VOLTAGE = 11.8;
    String shooterMode = "OFF";

    // ===== TA → DISTANCE (LOG REGRESSION) =====
    private double smoothedDistance = 0;
    // Reject bad TA
    private static final double MIN_TA = 0.2;
    // Shooter safety limits
    private static final double MIN_SHOOTER_RPM = 1175;
    private static final double MAX_SHOOTER_RPM = 2400;


    @Override
    public void runOpMode() throws InterruptedException {

        // ---- DRIVE MOTORS ----
        RF = hardwareMap.get(DcMotor.class, "RF");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        LB = hardwareMap.get(DcMotor.class, "LB");

        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.FORWARD);
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---- OTHER HARDWARE ----
        intake = hardwareMap.get(DcMotor.class, "Intake");
        left_Transfer = hardwareMap.get(CRServo.class, "Left Transfer");
        right_Transfer = hardwareMap.get(CRServo.class, "Right Transfer");
        hood = hardwareMap.get(Servo.class, "hood");
        led = hardwareMap.get(LED.class, "LedG");

        // ---- SHOOTER ----
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        // Get shooter controller for PIDF updates
        DcMotorControllerEx motorControllerEx = (DcMotorControllerEx) shooter.getController();
        int motorIndex = shooter.getPortNumber();

        // ---- LIMELIGHT ----
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag detection
        limelight.start();


        // ---- IMU ----
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )));
        imu.resetYaw();

        telemetry.addLine("READY - Hold LEFT TRIGGER to Auto Aim");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ========== MECANUM DRIVE ==========
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            double deadzone = 0.05;
            if (Math.abs(y) < deadzone) y = 0;
            if (Math.abs(x) < deadzone) x = 0;
            if (Math.abs(rx) < deadzone) rx = 0;

            // Auto-aim with Limelight

            boolean autoTargetActive = false;
            if (gamepad1.left_trigger > 0.3) {
                if (smoothedDistance > 90) {
                    shooter.setDirection(DcMotorSimple.Direction.FORWARD);
                    targetVelocity = longShooterVelocityFromDistance(smoothedDistance);
                    shooter.setVelocity(targetVelocity);
                    shooterMode = "AUTO (TA LOG)";
                }
                else if (smoothedDistance > 0) {
                    shooter.setDirection(DcMotorSimple.Direction.FORWARD);
                    targetVelocity = shortShooterVelocityFromDistance(smoothedDistance);
                    shooter.setVelocity(targetVelocity);
                    shooterMode = "AUTO (TA LOG)";
                }


                double rotationCorrection = getRotationCorrection();
                if (rotationCorrection != 0) {
                    rx = rotationCorrection;
                    autoTargetActive = true;
                } else {
                    lastError = 0;
                }
            } else {
                lastError = 0;
            }

            x *= 1.1;
            double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            LF.setPower((y + x + rx) / denom * (gamepad1.right_bumper ? 0.5 : 1));
            LB.setPower((y - x + rx) / denom * (gamepad1.right_bumper ? 0.5 : 1));
            RF.setPower((y - x - rx) / denom * (gamepad1.right_bumper ? 0.5 : 1));
            RB.setPower((y + x - rx) / denom * (gamepad1.right_bumper ? 0.5 : 1));

            // ========== INTAKE ==========
            if (gamepad2.left_trigger > 0.3) {
                intake.setDirection(DcMotorSimple.Direction.FORWARD);
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

            // ========== TRANSFER / CLEARING ==========
            if (gamepad2.triangle) {
                left_Transfer.setPower(1);
                right_Transfer.setPower(-1);
            } else if (gamepad2.cross) {
                left_Transfer.setPower(0);
                right_Transfer.setPower(0);
                shooter.setVelocity(0);
                intake.setPower(0);
                shooterMode = "OFF";
            } else if (gamepad2.circle) {
                left_Transfer.setPower(-1);
                right_Transfer.setPower(1);
                shooter.setDirection(DcMotorSimple.Direction.REVERSE);
                shooter.setVelocity(1200);
                intake.setDirection(DcMotorSimple.Direction.REVERSE);
                intake.setPower(.6);
            } else if (gamepad2.square) {
                left_Transfer.setPower(0);
                right_Transfer.setPower(0);
            }

            // ========== SHOOTER MODES ==========
            double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            double voltageCompF = NOMINAL_VOLTAGE / voltage;
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double ta = result.getTa();
                double rawDistance = distanceFromTA(ta);

                if (rawDistance > 0) {
                    smoothedDistance = (smoothedDistance == 0)
                            ? rawDistance
                            : 0.8 * smoothedDistance + 0.2 * rawDistance;
                }

                telemetry.addData("TA", "%.2f", ta);
                telemetry.addData("Distance (in)", "%.1f", smoothedDistance);
            }

            if (gamepad2.dpad_up) {
                targetVelocity = LONG_RANGE_VELOCITY;
                shooterMode = "LONG RANGE";
                sleep(150);
                hood.setPosition(.15);
                motorControllerEx.setPIDFCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDFCoefficients(LONG_P, LONG_I, LONG_D, LONG_F ));
            }

            if (gamepad2.dpad_down) {
                targetVelocity = SHORT_RANGE_VELOCITY;
                shooterMode = "SHORT RANGE";
                sleep(150);
                hood.setPosition(.06);
                motorControllerEx.setPIDFCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDFCoefficients(SHORT_P, SHORT_I, SHORT_D, SHORT_F ));
            }

            // Run shooter
            if (gamepad2.right_trigger > 0.3) {
                shooter.setDirection(DcMotorSimple.Direction.FORWARD);
                shooter.setVelocity(targetVelocity);
            } else if (gamepad2.right_bumper) {
                shooter.setVelocity(0);
                shooterMode = "OFF";
            }

            // LED shows if shooter is at 80%+ of target
            if (shooter.getVelocity() >= 0.8 * targetVelocity && targetVelocity > 0) {
                led.enableLight(true);
            } else {
                led.enableLight(false);
            }

            // ========== TELEMETRY ==========
            telemetry.addLine("");
            telemetry.addData("Drive Mode", autoTargetActive ? "AUTO-TARGETING" : "MANUAL");
            telemetry.addData("IMU Heading", "%.2f°", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addLine("");
            telemetry.addData("Shooter", shooterMode);
            telemetry.addData("Shooter TPS", "%.0f / %.0f", shooter.getVelocity(), targetVelocity);
            telemetry.update();
        }

        limelight.stop();
    }

    // =================== SAFE AUTO-AIM PD ===================
    private double getDistanceFromAngle(double ta) {
        return 1.0;
    }

    private double getRotationCorrection() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return 0;

        List<FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return 0;

        FiducialResult targetTag = null;
        for (FiducialResult tag : tags) {
            if (tag.getFiducialId() == 20) {
                targetTag = tag;
                break;
            }
        }
        if (targetTag == null) return 0;

        double error = result.getTx()*-1 + rotationOffset;

        // Dynamic proportional gain: slower near center
        double kP = Math.abs(error) > 15 ? 0.035 : 0.025;
        double rotationPower = error * kP;

        // Clamp rotation power
        double maxRotation = Math.max(minPower, Math.min(Math.abs(rotationPower), maxPower));
        rotationPower = Math.signum(rotationPower) * Math.min(Math.abs(rotationPower), maxRotation);

        // Stop if within tolerance
        if (Math.abs(error) < tolerance) rotationPower = 0;

        lastError = error;
        return -rotationPower;
    }

    private double distanceFromTA(double ta) {
        if (ta <= MIN_TA) return -1;

        // === DESMOS LOG REGRESSION (DO NOT CHANGE FORM) ===
        double distance = 72.53435 - 37.74489 * Math.log(ta);

        // Clamp to realistic field distances (inches)
        return Math.max(15, Math.min(130, distance));
    }

    private double longShooterVelocityFromDistance(double dist) {
        // === INITIAL FIT (TUNE THESE) ===
        double slope = 4.18;   // RPM per inch
        double intercept = 1030;

        double rpm = slope * dist + intercept;

        return Math.max(MIN_SHOOTER_RPM, Math.min(MAX_SHOOTER_RPM, rpm));
    }
    private double shortShooterVelocityFromDistance(double dist) {
        // === INITIAL FIT (TUNE THESE) ===
        double slope = 3.9;   // RPM per inch
        double intercept = 1030;

        double rpm = slope * dist + intercept;

        return Math.max(MIN_SHOOTER_RPM, Math.min(MAX_SHOOTER_RPM, rpm));
    }



}

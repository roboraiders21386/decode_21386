package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "Meet2_2Drivers_KG_EncoderShooter")
@Disabled
public class Meet2_2Drivers_KG_EncoderShooter extends LinearOpMode {

    private DcMotor RF, LF, RB, LB;
    private DcMotor intake;

    // Shooter now uses DcMotorEx for encoder + velocity control
    private DcMotorEx shooter;

    private CRServo right_Transfer;
    private CRServo left_Transfer;
    private Servo kicker;

    // shooter velocity target (ticks/sec)
    double targetVelocity = 1200;
    double increment = 75;

    final double NOMINAL_VOLTAGE = 12.0;
    double compensated;

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
        kicker = hardwareMap.get(Servo.class, "kicker");

        // ---- SHOOTER (ENCODER + VELOCITY) ----
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            // ----------- MECANUM DRIVE -----------
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            double deadzone = 0.05;
            if (Math.abs(y)  < deadzone) y  = 0;
            if (Math.abs(x)  < deadzone) x  = 0;
            if (Math.abs(rx) < deadzone) rx = 0;

            x *= 1.1; // strafe correction

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double frontLeft  = (y + x + rx) / denominator;
            double backLeft   = (y - x + rx) / denominator;
            double frontRight = (y - x - rx) / denominator;
            double backRight  = (y + x - rx) / denominator;

            double speed = gamepad1.right_bumper ? 0.4 : 1.0;

            LF.setPower(frontLeft  * speed);
            LB.setPower(backLeft   * speed);
            RF.setPower(frontRight * speed);
            RB.setPower(backRight  * speed);

            // ----------- INTAKE -----------
            if (gamepad2.left_trigger > 0.3) {
                intake.setDirection(DcMotorSimple.Direction.FORWARD);
                intake.setPower(1);
            } else if (!gamepad2.circle && !gamepad2.square) {
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

            } else if (gamepad2.circle) {   // full reverse clear
                left_Transfer.setPower(-1);
                right_Transfer.setPower(1);

                shooter.setDirection(DcMotorSimple.Direction.REVERSE);
                shooter.setVelocity(1200);

                intake.setDirection(DcMotorSimple.Direction.REVERSE);
                intake.setPower(1);

            } else if (gamepad2.square) {   // feed forward
                shooter.setDirection(DcMotorSimple.Direction.FORWARD);

                left_Transfer.setPower(1);
                right_Transfer.setPower(-1);

                intake.setDirection(DcMotorSimple.Direction.FORWARD);
                intake.setPower(1);
            }

            // ----------- SHOOTER (ENCODER + VOLTAGE COMP) -----------

            // get battery voltage
            double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            double voltageComp = NOMINAL_VOLTAGE / voltage;

            // enable shooter
            if (gamepad2.right_trigger > 0.3) {
                compensated = targetVelocity * voltageComp;
                shooter.setDirection(DcMotorSimple.Direction.FORWARD);
                shooter.setVelocity(compensated);

            } else if (gamepad2.right_bumper) {
                shooter.setVelocity(0);
            }

            // adjust target velocity using dpad
            if (gamepad2.dpad_up) {
                targetVelocity += increment;
                sleep(120);
            }

            if (gamepad2.dpad_down) {
                targetVelocity -= increment;
                if (targetVelocity < 0) targetVelocity = 0;
                sleep(120);
            }

            // ----------- KICKER -----------
            if (gamepad2.dpad_right) {
                kicker.setPosition(1.0);
                sleep(250);
                kicker.setPosition(0.0);
            }

            // ----------- TELEMETRY FOR TUNING -----------
            telemetry.addData("Battery Voltage", voltage);
            telemetry.addData("Target TPS", targetVelocity);
            //telemetry.addData("Current TPS");
            telemetry.addData("Shooter Velocity", shooter.getVelocity());
            telemetry.addData("Shooter Power:", compensated);
            telemetry.update();
        }
    }
}
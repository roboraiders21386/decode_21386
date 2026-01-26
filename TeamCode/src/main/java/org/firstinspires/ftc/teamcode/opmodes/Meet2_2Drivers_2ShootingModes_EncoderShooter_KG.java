package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "Meet2_2Drivers_2ShootingModes_Android Studio")
public class Meet2_2Drivers_2ShootingModes_EncoderShooter_KG extends LinearOpMode {

    // Shooter mode target velocity
    double targetVelocity = 0;

    // Preset shooter velocities (ticks/sec)
    final double LONG_RANGE_VELOCITY  = 1800;
    final double SHORT_RANGE_VELOCITY = 1475;

    final double NOMINAL_VOLTAGE = 12.0;
    double P;
    double F;

    String shooterMode = "OFF";

    @Override
    public void runOpMode() throws InterruptedException {

        // ---- DRIVE MOTORS ----
        DcMotor RF = hardwareMap.get(DcMotor.class, "RF");
        DcMotor LF = hardwareMap.get(DcMotor.class, "LF");
        DcMotor RB = hardwareMap.get(DcMotor.class, "RB");
        DcMotor LB = hardwareMap.get(DcMotor.class, "LB");

        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.FORWARD);
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---- OTHER HARDWARE ----
        DcMotor intake = hardwareMap.get(DcMotor.class, "Intake");
        CRServo left_Transfer = hardwareMap.get(CRServo.class, "Left Transfer");
        CRServo right_Transfer = hardwareMap.get(CRServo.class, "Right Transfer");

        // ---- SHOOTER (ENCODER + VELOCITY MODE) ----
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

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
            double compensated = targetVelocity * voltageComp;

            if(gamepad1.dpadUpWasPressed()) {
                if(targetVelocity == LONG_RANGE_VELOCITY){
                    targetVelocity = SHORT_RANGE_VELOCITY;
                    P = 20.303;
                    F = 21.91;
                    shooter.setVelocity(compensated);
                }
                if(gamepad1.dpad_down) {
                    targetVelocity = LONG_RANGE_VELOCITY;
                    P=137.41;
                    F=270.2;
                    shooter.setVelocity(compensated);
                }
            }

            // Select mode
            if (gamepad2.dpad_up) {
                targetVelocity = LONG_RANGE_VELOCITY;
                shooterMode = "LONG RANGE";
                sleep(150);
            }

            if (gamepad2.dpad_down) {
                targetVelocity = SHORT_RANGE_VELOCITY;
                shooterMode = "SHORT RANGE";
                sleep(150);
            }

            // Run shooter when trigger is held
            if (gamepad2.right_trigger > 0.3) {
                shooter.setDirection(DcMotorSimple.Direction.FORWARD);
                shooter.setVelocity(compensated);

            } else if (gamepad2.right_bumper) {
                shooter.setVelocity(0);
                shooterMode = "OFF";
            }


            // ----------- TELEMETRY -----------
            telemetry.addData("Shooter Mode", shooterMode);
            telemetry.addData("Battery Voltage", voltage);
            telemetry.addData("Target TPS", targetVelocity);
            telemetry.addData("Actual TPS", shooter.getVelocity());
            telemetry.update();
        }
    }
}

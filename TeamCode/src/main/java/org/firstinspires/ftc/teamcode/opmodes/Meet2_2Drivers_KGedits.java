package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;


@TeleOp(name = "Meet2_2Drivers_KGedits")
@Disabled
public class Meet2_2Drivers_KGedits extends LinearOpMode {
    /*
    private DcMotor RF, LF, RB, LB;
    private DcMotor shooter, intake;
    private CRServo left_Transfer, right_Transfer;
    */
    private DcMotor RF;
    private DcMotor LF;
    private DcMotor RB;
    private DcMotor LB;
    private double dp = 1;
    private double sp = .925;
    private double ip = 1;
    private double zp = 0;
    private CRServo right_Transfer;
    private CRServo left_Transfer;
    private DcMotor intake;
    private DcMotor shooter;
    private Servo kicker;


    @Override
    public void runOpMode() throws InterruptedException {

        // ---- MOTOR SETUP ----
        RF = hardwareMap.get(DcMotor.class, "RF");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        LB = hardwareMap.get(DcMotor.class, "LB");

        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.FORWARD);
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter = hardwareMap.get(DcMotor.class, "Shooter");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        left_Transfer = hardwareMap.get(CRServo.class, "Left Transfer");
        right_Transfer = hardwareMap.get(CRServo.class, "Right Transfer");
        kicker = hardwareMap.get(Servo.class, "kicker");

        waitForStart();

        while (opModeIsActive()) {

            // ---------------- MECANUM DRIVE ----------------
            /*double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeft  = (y + x + rx) / denominator;
            double backLeft   = (y - x + rx) / denominator;
            double frontRight = (y - x - rx) / denominator;
            double backRight  = (y + x - rx) / denominator;

            LF.setPower(frontLeft);
            LB.setPower(backLeft);
            RF.setPower(frontRight);
            RB.setPower(backRight);

            */
            // -------------------- MECANUM DRIVE --------------------

            // Gamepad values
            double y  = -gamepad1.left_stick_y;   // forward/back (FTC inverted)
            double x  =  gamepad1.left_stick_x;   // strafe (DO NOT invert)
            double rx = -gamepad1.right_stick_x;  // turn

            // Deadzone
            double deadzone = 0.05;
            if (Math.abs(y)  < deadzone) y  = 0;
            if (Math.abs(x)  < deadzone) x  = 0;
            if (Math.abs(rx) < deadzone) rx = 0;

            // Optional strafe correction
            double strafeMultiplier = 1.1;
            x *= strafeMultiplier;

            // Normalization
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            // Mecanum math (STANDARD)
            double frontLeft  = (y + x + rx) / denominator;
            double backLeft   = (y - x + rx) / denominator;
            double frontRight = (y - x - rx) / denominator;
            double backRight  = (y + x - rx) / denominator;

            // Slow mode (optional)
            double speed = gamepad1.right_bumper ? 0.4 : 1.0;

            // Apply power
            LF.setPower(frontLeft  * speed);
            LB.setPower(backLeft   * speed);
            RF.setPower(frontRight * speed);
            RB.setPower(backRight  * speed);


            // ---------------- INTAKE ----------------
            if (gamepad2.left_trigger > 0.3) {
                intake.setDirection(DcMotorSimple.Direction.FORWARD);
                intake.setPower(1);
            }
            if (gamepad2.left_trigger < 0.3 && !gamepad2.circle && !gamepad2.square) {
                intake.setPower(0);
            }


            /*
            // ---------------- TRANSFER ----------------
            if (gamepad2.triangle) {
                left_Transfer.setPower(-1);
                right_Transfer.setPower(1);
            }
            else if (gamepad2.circle) {
                left_Transfer.setPower(1);
                right_Transfer.setPower(-1);
            }
            else if (gamepad2.cross) {
                left_Transfer.setPower(0);
                right_Transfer.setPower(0);
            }


            // ---------------- SHOOTER ----------------
            if (gamepad2.right_trigger > 0) {
                shooter.setDirection(DcMotorSimple.Direction.FORWARD);
                shooter.setPower(shooterPower);
            }
            else if (gamepad2.right_bumper) {
                shooter.setPower(0);
            }

            // Adjust shooter speed
            if (gamepad2.dpad_up) {
                shooterPower = Math.min(shooterPower + 0.02, 1.0);
            }
            else if (gamepad2.dpad_down) {
                shooterPower = Math.max(shooterPower - 0.02, 0);
            }
            */
            // ------- TRANSFER -------
            if (gamepad2.triangle) { //push up
                left_Transfer.setPower(1);
                right_Transfer.setPower(-1);
            } else if (gamepad2.cross) {//stop transfer
                left_Transfer.setPower(0);
                right_Transfer.setPower(0);
                intake.setPower(0);
            }else if (gamepad2.circle) {//push down
                left_Transfer.setPower(-1);
                right_Transfer.setPower(1);
                shooter.setDirection(DcMotorSimple.Direction.REVERSE);
                shooter.setPower(0.35);//shooter going slowly, square the ball in the center
                intake.setDirection(DcMotorSimple.Direction.REVERSE);
                intake.setPower(1);
            }else if (gamepad2.square) {
                shooter.setDirection(DcMotorSimple.Direction.REVERSE);
                shooter.setPower(sp);

                left_Transfer.setPower(1);
                right_Transfer.setPower(-1);

                intake.setDirection(DcMotorSimple.Direction.FORWARD);
                intake.setPower(1);
            }

            // ------- SHOOTER -------
            /*if (gamepad2.right_trigger > 0) {
                shooter.setPower(sp);
                shooter.setDirection(DcMotorSimple.Direction.FORWARD);

            } else if (gamepad2.dpad_up){
                shooter.setPower(sp+0.05);
            } else if (gamepad2.dpad_down){
                shooter.setPower(sp-0.05);
            } else if (gamepad2.right_bumper) {
                shooter.setPower(0);
            }*/
            // ------- SHOOTER -------
            if (gamepad2.right_trigger > 0.3) {
                shooter.setDirection(DcMotorSimple.Direction.FORWARD);
                shooter.setPower(sp);
                if (gamepad2.right_trigger < 0.2 && !gamepad2.circle) {
                    shooter.setPower(0);
                }
            } else if (gamepad2.dpad_up) {
                sp = Math.min(sp + 0.05, 1.0);   // <-- FIXED (updates sp now)
                shooter.setPower(sp);
                sleep(120);                      // prevent spam increments

            } else if (gamepad2.dpad_down) {
                sp = Math.max(sp - 0.05, 0.0);   // <-- FIXED (updates sp now)
                shooter.setPower(sp);
                sleep(120);

            } else if (gamepad2.right_bumper) {
                shooter.setPower(0);
            }
            //-------KICKER----------
            if(gamepad2.dpad_right){
                kicker.setPosition(-2000);
                sleep(1000);
                kicker.setPosition(100);
            }

            // ----------- TELEMETRY FOR TUNING -----------
            telemetry.addData("Shooter Power", sp);
            telemetry.update();

        }
    }
}
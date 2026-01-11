package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "Meet2_TeleOp")
public class Meet2_TeleOp extends LinearOpMode {

    private DcMotor RF, LF, RB, LB;
    private DcMotor intake;
    private DcMotorEx shooter;

    private CRServo right_Transfer;
    private CRServo left_Transfer;
    private Servo hood;
    

    
    //color sensor logic
    private NormalizedColorSensor LcolorSensor;
    private NormalizedColorSensor RcolorSensor;
    ElapsedTime detectionTimerLeft = new ElapsedTime();
    boolean isArtifactPresentLeft = false;
    ElapsedTime detectionTimerRight = new ElapsedTime();
    boolean isArtifactPresentRight = false;
    
    // Shooter mode target velocity
    double targetVelocity = 0;

    // Preset shooter velocities (ticks/sec)
    final double LONG_RANGE_VELOCITY  = 1800;
    final double SHORT_RANGE_VELOCITY = 1300;

    final double NOMINAL_VOLTAGE = 12.0;
    
    public static final double SHORT_P = 12.1, SHORT_F = 16.6, LONG_P = 12.4, LONG_F = 14.6;
    String shooterMode = "OFF"; 

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
        //led = hardwareMap.get(LED.class, "LedG");

        // ---- SHOOTER (ENCODER + VELOCITY MODE) ----
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        
        /*//color sensors
        LcolorSensor = hardwareMap.get(NormalizedColorSensor.class, "leftColorSensor");
        RcolorSensor = hardwareMap.get(NormalizedColorSensor.class, "rightColorSensor");
        // Try higher gain values (2-15 range)
        LcolorSensor.setGain(15); // Start with maximum gain
        RcolorSensor.setGain(15);*/


        waitForStart();
        DcMotorControllerEx motorControllerEx = (DcMotorControllerEx)shooter.getController();
        int motorIndex = ((DcMotorEx)shooter).getPortNumber();
        PIDFCoefficients pidfshort = new PIDFCoefficients(SHORT_P, 0, 0, SHORT_F);
        PIDFCoefficients pidflong = new PIDFCoefficients(LONG_P, 0, 0, LONG_F);
        while (opModeIsActive()) {
            // color sensor initialization
            /*NormalizedRGBA Lcolors = LcolorSensor.getNormalizedColors();
            NormalizedRGBA colorsR = RcolorSensor.getNormalizedColors();
            
            // Process Sensor 1
            if (Lcolors.alpha > 0.01) {
                float hueL = JavaUtil.colorToHue(Lcolors.toColor());
                telemetry.addData("Left Color Sensor Hue", "%.2f", hueL);
                
                boolean isGreenL = (hueL >= 90 && hueL <= 150);
                boolean isPurpleL = (hueL >= 180 && hueL <= 300);
                boolean objectDetectedL = (isGreenL || isPurpleL) && (Lcolors.alpha > 0.1);
                
                if (objectDetectedL) {
                    if (!isArtifactPresentLeft) {
                        detectionTimerLeft.reset();
                        isArtifactPresentLeft = true;
                    }
                    telemetry.addData("Left Sensor Time", "%.1f", detectionTimerLeft.seconds());
                } else {
                    isArtifactPresentLeft = false;
                    detectionTimerLeft.reset();
                }
            } else {
                telemetry.addData("Left Sensor", "No light detected");
                isArtifactPresentLeft = false;
            }
            
            // Process Right Sensor
            if (colorsR.alpha > 0.01) {
                float hueR = JavaUtil.colorToHue(colorsR.toColor());
                telemetry.addData("Right Sensor Hue", "%.2f", hueR);
                
                boolean isGreenR = (hueR >= 90 && hueR <= 150);
                boolean isPurpleR = (hueR >= 180 && hueR <= 300);
                boolean objectDetectedR = (isGreenR || isPurpleR) && (colorsR.alpha > 0.1);
                    
                    if (objectDetectedR) {
                        if (!isArtifactPresentRight) {
                            detectionTimerRight.reset();
                            isArtifactPresentRight = true;
                        }
                        telemetry.addData("Right Sensor Time", "%.1f", detectionTimerRight.seconds());
                    } else {
                        isArtifactPresentRight = false;
                        detectionTimerRight.reset();
                    }
                } else {
                    telemetry.addData("Right ", "No light detected");
                    isArtifactPresentRight = false;
                }
                
                // Combined Logic - Stop motor if EITHER sensor detects for 3 seconds
                boolean LeftSensorReady = isArtifactPresentLeft && (detectionTimerLeft.seconds() >= 3.0);
                boolean RightSensorReady = isArtifactPresentRight && (detectionTimerLeft.seconds() >= 3.0);
                
                if (LeftSensorReady || RightSensorReady) {
                    intake.setPower(0);
                    telemetry.addLine("artifact detected!");
                    
                    if (LeftSensorReady)
                        telemetry.addLine("  Triggered by Left Sensor");
                    if (RightSensorReady)
                        telemetry.addLine("  Triggered by Right Sensor");
                }*/
            
        

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
            
            // ----------- TELEMETRY -----------
            //telemetry.addLine("=== REGRESSION SHOOTER (Hood 0-0.2) ===");
            //telemetry.addData("Hood Angle (0-0.2)", "%.3f", hoodAngle);
            /*telemetry.addLine("---------Color Sensor--------------");
            telemetry.addData("left Alpha (brightness)", "%.3f", Lcolors.alpha);
            telemetry.addData("left Red", "%.3f", Lcolors.red);
            telemetry.addData("left Green", "%.3f", Lcolors.green);
            telemetry.addData("left Blue", "%.3f", Lcolors.blue);
            telemetry.addData("right Alpha (brightness)", "%.3f", colorsR.alpha);
            telemetry.addData("right Red", "%.3f", colorsR.red);
            telemetry.addData("right Green", "%.3f", colorsR.green);
            telemetry.addData("right Blue", "%.3f", colorsR.blue);*/
            telemetry.addLine("---------Shooter--------------");
            telemetry.addData("Shooter Mode", shooterMode);
            telemetry.addData("Battery Voltage", voltage);
            telemetry.addData("Target TPS", targetVelocity);
            telemetry.addData("Actual TPS", shooter.getVelocity());
            telemetry.update();
        }
    }
}

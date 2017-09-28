package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

/**
 * Created by Alex on 9/18/2017.
 *  *
 * NOTE:
 *
 * This is intended to be the master autonomous file. Place all methods that you write into this
 * file so they can be inherited by other autonomous files. By using this approach, we can increase
 * the readability of autonomous files and have a database of various methods so they do not have
 * to be written down in each one.
 */

@Disabled
@Autonomous(name = "Method Master", group = "Master")
public class AutonomousMethodMaster extends LinearOpMode {

    /** Declaring the motor variables **/
    /** ---------------------------------------------------------------------------------------- **/
    private DcMotor motorL;                       // Left Side Motor
    private DcMotor motorR;                       // Right Side Motor
    /** ---------------------------------------------------------------------------------------- **/
    /** For Encoders and specific turn values **/
    /* ------------------------------------------------------------------------------------------ */
    double ticksPerRev = 1120;             // This is the specific value for AndyMark motors
    double ticksPerRevTetrix = 1440;       // The specific value for Tetrix, since only one encoded Tetrix motor (launcher arm)
    double ticksPer360Turn = 4500;         // The amount of ticks for a 360 degree turn for AnyMarks
    double tickTurnRatio = ticksPer360Turn / 360;
    double inchToMm = 25.4;             // For conversion between the vectors

    double wheelDiameter = 4.0;         // Diameter of the current omniwheels in inches
    double ticksPerInch = (ticksPerRev / (wheelDiameter * 3.14159265));

    double encoderResetSpeed = 0.25;        // Motor speed for when the robot resets launcher
    /* ------------------------------------------------------------------------------------------ */


    public void runOpMode() throws InterruptedException {
        /** This is the method that executes the code and what the robot should do **/
        // Call any variables not stated before

        // Initializes the electronics
        initElectronics(0);

        telemetry.addData("Phase 1", "Init");
        telemetry.update();

        waitForStart();

        telemetry.addData("Started Robot", "Now");
        telemetry.update();

        runToPositionEncoders();

    }

    /**
     * These methods control the encoder modes of the motor
     **/
    /** ----------------------------------------- **/
    public void encoderMode(int mode) {
        /**NOTE:
         *  This was made just for the sake of making the code look a bit neater
         *
         * Mode Numbers:
         *  0 = RUN_TO_POSITION
         *  1 = RUN_USING_ENCODER
         *  2 = RUN_WITHOUT_ENCODER
         *  3 = STOP_AND_RESET_ENCODERS
         *  4 = RESET_ENCODERS
         *  **/
        if (mode == 0) {
            /** Sets the encoded motors to RUN_TO_POSITION **/
            motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (mode == 1) {
            /** Sets the encoders to RUN_USING_ENCODERS **/
            motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if (mode == 2) {
            /** Sets the encoders to RUN_WITHOUT_ENCODERS **/
            motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else if (mode == 3) {
            /** Stops and resets the encoder values on each of the drive motors **/
            motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else if (mode == 4) {
            /** Resets the encoder values on each of the drive motors **/
            motorL.setMode(DcMotor.RunMode.RESET_ENCODERS);
            motorR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        }
    }

    public void resetEncoders() throws InterruptedException {
        /** Resets the encoder values on each of the drive motors **/
        motorL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorR.setMode(DcMotor.RunMode.RESET_ENCODERS);
//        waitOneFullHardwareCycle();
    }

    public void runToPositionEncoders() {
        /** Sets the encoded motors to RUN_TO_POSITION **/
        motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runUsingEncoders() {
        /** Sets the encoders to RUN_USING_ENCODERS **/
        motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void addTelemetryData(String string1, String string2) {
        telemetry.addData(string1, string2);
        telemetry.update();
    }

    public void stopMotion() {
        /** Stops all drive motor motion **/
        motorL.setPower(0);
        motorR.setPower(0);
    }
    /** ----------------------------------------- **/


    /**
     * These methods are used to set up the robot
     **/
    /** ----------------------------------------- **/
    public void initElectronics(int mode) throws InterruptedException {
        // To make the initialization of electronics much easier and nicer to read
        /** Initializing and mapping electronics **/
        if (mode == 0) {
            /* Motors and servos (w/ controllers) */
            // motorControllerL = hardwareMap.dcMotorController.get("MC_L");
            // motorControllerR = hardwareMap.dcMotorController.get("MC_R");
            // motorControllerA1 = hardwareMap.dcMotorController.get("MC_A1");
            // motorControllerA2 = hardwareMap.dcMotorController.get("MC_A2");
            // servoController = hardwareMap.servoController.get("SC");

            motorL = hardwareMap.dcMotor.get("motorL");        //P0 is actually the right
            motorR = hardwareMap.dcMotor.get("motorR");        //P1 is actually the left

            // servo = hardwareMap.servo.get("servo");

            // motorLauncher = hardwareMap.dcMotor.get("motorLauncher");   //P0
            // sweeperMotor = hardwareMap.dcMotor.get("motorSweeper");     //P1

            // motorStrafe = hardwareMap.dcMotor.get("motorStrafe");       //P0 A2



            /* Sensors */
            // colorBeacon = hardwareMap.colorSensor.get("colorBeacon");



            /*Setting channel modes*/
            runUsingEncoders();

            // motorLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // sweeperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // motorStrafe.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Since the motors face different directions, one is going to go in an opposite direction to the other.
            // This is intended to correct that problem and will be adjusted later as we do testing and the true directions are recorded
            motorL.setDirection(DcMotorSimple.Direction.REVERSE);

            // motorLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }


    public void encoderMove(double power, double leftInches, double rightInches) {
        /** This method makes the motors move a certain distance **/
        // resetEncoders();

        // Sets the power range
        power = Range.clip(power, -1, 1);

        // Setting the target positions
        motorL.setTargetPosition((int)(leftInches * -ticksPerInch));
        motorR.setTargetPosition((int)(rightInches * -ticksPerInch));

        runToPositionEncoders();

        // Sets the motors' position
        motorL.setPower(power);
        motorR.setPower(power);

        // While loop for updating telemetry
        while(motorL.isBusy() && motorR.isBusy() && opModeIsActive()){

            // Updates the position of the motors
            double LPos = motorL.getCurrentPosition();
            double RPos = motorR.getCurrentPosition();

            // Adds telemetry of the drive motors
            telemetry.addData("MotorL Pos", LPos);
            telemetry.addData("MotorR Pos", RPos);

            // Updates the telemetry
            telemetry.update();

        }

        // Stops the motors
        stopMotion();

        // Resets to run using encoders mode
        runUsingEncoders();

    }


    /** ----------------------------------------- **/
}
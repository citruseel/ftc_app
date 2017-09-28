/* 
 * V 2.0 - 
 * Add fuctionality of changing the power given to motors based on state of the right bumper
 * pressed makes gear ratio .7 while non pressed makes gear ratio .3
 * 
 * V 1.0 - Loops through and sets power to motors based on joystick position
 */


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by user on 9/22/2017.
 */

@Disabled
@TeleOp(name="BasicTeleop", group = "Basic")
public class BasicTeleop extends OpMode
{
    // Initialize the components of the robot
    /* ---------------------------------------- */
    private DcMotorController motorControllerDrive;
    private DcMotor motorR, motorL;
    /* ---------------------------------------- */


    @Override
    public void init()
    {   //Assign values to hardware components here (match them to phone configuration)
        // Motor and motor controller hardware declaration
        /* ---------------------------------------- */
        motorControllerDrive = hardwareMap.dcMotorController.get("MC_D");

        motorR = hardwareMap.dcMotor.get("motorR");
        motorL = hardwareMap.dcMotor.get("motorL");
        /* ---------------------------------------- */

        // Encoder stuff
        /* ---------------------------------------- */
        motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        motorR.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorL.setDirection(DcMotorSimple.Direction.REVERSE);
        /* ---------------------------------------- */

    }

    public void loop()
    {   /*
         * Gets position of the joysticks on controller1
         * and sets the power of the motors as the position multiplied by a constant
         * to influence the potency of the motors.
         */
        double rightPower, leftPower;
        double gearRatio = gamepad1.right_bumper ? 0.7 : 0.2;
        // If right_bumper is down, the gearRatio is 0.7. Otherwise, the gearRatio is 0.3
        
        rightPower = gearRatio * gamepad1.left_stick_y;
        leftPower = gearRatio * gamepad1.right_stick_y;
        
        leftpower = Range.clip(leftpower, -1, 1);        //gamepad controllers have a value of 1 when you push it to its maximum foward
        rightpower = Range.clip(rightpower, -1, 1);      //limiting the range of each power, min first then max
        
        motorR.setPower(rightPower);
        motroL.setPower(leftPower);
          
        telemetry.addData("Gear Ratio ", gearRatio);
        telemetry.addData("Right Power ", rightPower);
        telemetry.addData("Left Power ", leftPower);
        
        
    }
}i

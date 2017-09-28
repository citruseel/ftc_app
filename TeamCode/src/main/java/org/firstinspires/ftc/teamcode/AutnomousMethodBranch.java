package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by citruseel on 9/27/2017.
 */
@Autonomous(name = "Autonomous Branch Test", group = "Branch")
public class AutnomousMethodBranch extends AutonomousMethodMaster {

    @Override
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

        encoderMove(1, 24, 24); //move forward 24 inches at power of 1
    }
}

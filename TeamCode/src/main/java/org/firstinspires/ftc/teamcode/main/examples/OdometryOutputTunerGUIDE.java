package org.firstinspires.ftc.teamcode.main.examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;

@Disabled
public class OdometryOutputTunerGUIDE extends LinearOpMode {

    // Our robot
    private OptimizedRobot robot;

    // Variables down here!



    // Create a non-final var that is something like distanceToTravel (in inches)


    // Create two vars called like estimated X and estimated Y and init them to 0



    @Override
    public void runOpMode() throws InterruptedException {

        // All code goes in this func!

        // Initializing our robot
        robot = new OptimizedRobot(gamepad1, gamepad2, telemetry, hardwareMap);

        // Grab our controller 1 using robot.getController1()



        // Grab our our encoders using robot.getEncoder() there are TWO of them
        // Use xEncoder and yEncoder for the names!


        // Reverse the direction of our left encoder using encoder.setDirection()


        // Waiting for the play button to be pressed
        waitForStart();


        // Print out some instructions using robot.log() or a method like that


        // Make a loop to wait until player presses A


        // Make a loop to wait till the player presses A again (and output something like press A when finished)
        // Also, update estimateX and estimatedY by the encoder.getCurrentPosition() in this loop


        // Record the estimatedX and Y from this trial (they moved the robot some distance forward and to the side)
        // Set those two vars back to zero, but make sure to record them!!! Maybe in an array?


        // Repeat this process another two times?


        // After the tuning, calculate the coefficient for both X and Y using the formula (actual / measured) for each of the trials
        // Average the X and average the Y for two final values: an X and a Y cooefficient
        // Prints those here!


        // Put a loop to keep the opmode running until they hit B or something


        // Once the ticks
    }
}

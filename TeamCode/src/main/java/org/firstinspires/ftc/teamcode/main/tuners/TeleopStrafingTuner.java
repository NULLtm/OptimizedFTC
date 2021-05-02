package org.firstinspires.ftc.teamcode.main.tuners;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;

@Autonomous(name="StrafingCoefficientTuner", group="TUNING")
public class TeleopStrafingTuner extends OpMode {


    // Robot
    private OptimizedRobot robot;

    // Variables

    // Define one double var for the amplitude of our sin func


    // define a double var called runtime here


    // define a double called start time here


    // Define some const for the speed (as a percentage) the robot should strafe at


    // Define a var for our controller1



    @Override
    public void init() {
        // Instantiating our robot
        robot = robot = new OptimizedRobot(gamepad1, gamepad2, telemetry, hardwareMap);


        // Init our controller using robot.getControl1 or some method like that


        // set our start time var to runTime()
    }

    @Override
    public void loop() {

        // set our runtime var to the difference between our start time and runTime()
        // aka finding the time since starting!


        // find our strafing coEfficient using A*Sin(x) where A is our amplitude var from above


        // Call updateDrive() with our parameters



        // Check if A is pressed
    }



    // Fill this method here with the teleop code, found in updateDrive()
    private void updateDrive(double baseSpeed, double strafingCo) {

    }
}

package org.firstinspires.ftc.teamcode.main.examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.teamcode.internal.OptimizedDriveFunctions;
import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;


/**
 * Example of an autonomous opmode
 *
 * NOTE: Autonomous op modes must have the @autonomous annotation and must extend from LinearOpMode
 */
public class ExampleAutonomous extends LinearOpMode {

    // Robot instance
    private OptimizedRobot robot = null;

    @Override
    public void runOpMode() {
        // Initialization Area

        // This line is constant among every opmode. Just copy and paste honestly if you don't understand this
        robot = new OptimizedRobot(gamepad1, gamepad2, telemetry, hardwareMap);



        // Initializing our drive train motors. This is REQUIRED in an autonomous OpMode.
        // Look in OptimizedRobot at the different overloaded versions of this method that do different things.
        robot.initializeDriveMotors(true);



        // Pauses until the driver hits the play button
        waitForStart();





        // Logging to console on the phone
        robot.log("Message:", "Hello Drivers!");




        robot.getDriveFunctions().strafeLinear(OptimizedDriveFunctions.Direction.LEFT, 0.75);
        sleep(2000);
        robot.getDriveFunctions().stopMotors();
    }
}

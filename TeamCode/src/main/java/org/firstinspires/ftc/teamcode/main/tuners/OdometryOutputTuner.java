package org.firstinspires.ftc.teamcode.main.tuners;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.internal.OptimizedController;
import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;
import org.firstinspires.ftc.teamcode.util.Encoder;


/**
 * NOT WORKING RN -- DO NOT USE
 */
@Deprecated
@Autonomous(name="OdometryOutputTuner", group = "TUNING")
public class OdometryOutputTuner extends LinearOpMode {

    // Our robot
    private OptimizedRobot robot;

    // Variables down here!

    private OptimizedController controller1;
    private Encoder xEncoder;
    private Encoder yEncoder;


    // Create a non-final var that is something like distanceToTravel (in inches)

    private double DISTANCE_TO_TRAVEL = 48; // in inches

    private int NUM_SAMPLES = 3;

    private final int TICKS_PER_REV = 8192;
    private final double ODOMETRY_WHEEL_RADIUS = 0.75;


    // Create two vars called like estimated X and estimated Y and init them to 0

    private int[] xTraveled = new int[NUM_SAMPLES];
    private int[] yTraveled = new int[NUM_SAMPLES];

    private int estimatedX = 0, estimatedY = 0;

    private int prevX = 0, prevY = 0;



    @Override
    public void runOpMode() throws InterruptedException {

        // All code goes in this func!

        // Initializing our robot
        robot = new OptimizedRobot(gamepad1, gamepad2, telemetry, hardwareMap);

        // Grab our controller 1 using robot.getController1()

        controller1 = robot.getController1();

        // Grab our our encoders using robot.getEncoder() there are TWO of them
        // Use xEncoder and yEncoder for the names!

        xEncoder = robot.getEncoder("XEncoder");
        yEncoder = robot.getEncoder("YEncoder");

        // Reverse the direction of our left encoder using encoder.setDirection()

        xEncoder.setDirection(Encoder.Direction.REVERSE);


        // Waiting for the play button to be pressed
        waitForStart();

        robot.log("Instructions", "Your job is the move the robot "+DISTANCE_TO_TRAVEL+" forward and right for each trial!");

        for(int i = 0; i < NUM_SAMPLES; i++) {
            robot.addLog("Prompt", "Trial #"+(i+1)+" of "+NUM_SAMPLES);
            robot.addLog("Prompt", "Set up your robot on its start position and press A to start the tuning!");
            robot.pushLog();

            while(!controller1.getBool(OptimizedController.Key.A)) {
            }

            prevX = xEncoder.getCurrentPosition();
            prevY = yEncoder.getCurrentPosition();

            robot.addLog("Running", "Drag your robot forward "+DISTANCE_TO_TRAVEL+" inches and "+DISTANCE_TO_TRAVEL+" inches to the right!");
            robot.addLog("Running", "Try to keep the robot straight the entire time--try using a line on the floor for reference.");
            robot.addLog("Instructions", "Press B when you have finished moving the robot!");
            robot.pushLog();

            while(!controller1.getBool(OptimizedController.Key.B)) {
                estimatedX = xEncoder.getCurrentPosition() - prevX;
                estimatedY = yEncoder.getCurrentPosition() - prevY;
            }

            xTraveled[i] = estimatedX;
            yTraveled[i] = estimatedY;
        }


        double xCoefficient = 0;
        double yCoefficient = 0;

        for(int num : xTraveled) {
            xCoefficient += (TICKS_PER_REV * (DISTANCE_TO_TRAVEL / (2 * Math.PI * ODOMETRY_WHEEL_RADIUS))) / num;
        }

        for(int num : yTraveled) {
            yCoefficient += (TICKS_PER_REV * (DISTANCE_TO_TRAVEL / (2 * Math.PI * ODOMETRY_WHEEL_RADIUS))) / num;
        }

        xCoefficient /= NUM_SAMPLES;
        yCoefficient /= NUM_SAMPLES;

        robot.addLog("xEncoder Coefficient = ", Math.abs(xCoefficient));
        robot.addLog("yEncoder Coefficient = ", Math.abs(yCoefficient));
        robot.pushLog();

        while(opModeIsActive()) {
        }
    }
}

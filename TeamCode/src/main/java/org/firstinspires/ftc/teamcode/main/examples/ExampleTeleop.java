package org.firstinspires.ftc.teamcode.main.examples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.internal.OptimizedController;
import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;
import org.firstinspires.ftc.teamcode.main.pipelines.FinalWABOTPipeline;

public class ExampleTeleop extends OpMode {

    // Our 'virtual' controllers we are using
    private OptimizedController controller1, controller2, controller3;

    // Our non-drivetrain motors we are using
    private DcMotor exampleMotor;

    // Our robot reference -- very important!
    private OptimizedRobot robot;

    @Override
    public void init() {
        // First thing you should do -- instantiate the robot
        robot = new OptimizedRobot(gamepad1, gamepad2, telemetry, hardwareMap, new SampleControllerMapping(), new SampleHardwareAliasMapping());

        // Set up our controllers -- to activate them: hit start+AorB to activate the gamepad and then, while still holding down START, press A, B, or X in this case.

        // Our main drive controller
        controller1 = robot.setUpVirtualController(null, OptimizedController.Key.A);
        // Our secondary drive controller
        controller2 = robot.setUpVirtualController(null, OptimizedController.Key.B);
        // Our end game controller with end game features
        controller3 = robot.setUpVirtualController(null, OptimizedController.Key.X);

        // Set up our vision detection using a custom pipeline
        // The first argument is whether you want the output to show on the screen!
        robot.initializeOpenCVPipeline(false, new FinalWABOTPipeline());

        // Set up RoadRunner for automatic end game scoring, if need be
        robot.initializeRoadRunner();

        // Set up our motor from before using an alias -- note this is a DcMotorEx used for setVelocity control
        exampleMotor = robot.getMotorExByAlias("myAlias", DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotorSimple.Direction.FORWARD);

        robot.teleopLog("Status Update", "Done with Initialization!");
        robot.pushLog();
    }

    @Override
    public void loop() {

        // Update our example motor to use a simple intake/outtake algorithm provided by our drive func class!
        robot.getDriveFunctions().updateTwoWayMotorController("intake", "outtake", exampleMotor, controller2);

        // Update our controllers so they can be initialized
        robot.updateVirtualControllerStartup();

        // Update our drive algorithm
        robot.updateDrive(controller1, controller2, true, true, 0.7, 0.5, 0.25, OptimizedRobot.RobotDirection.LEFT, OptimizedRobot.RobotDirection.RIGHT, true);
    }
}

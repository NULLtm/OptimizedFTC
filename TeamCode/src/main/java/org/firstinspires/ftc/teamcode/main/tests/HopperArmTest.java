package org.firstinspires.ftc.teamcode.main.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.internal.OptimizedController;
import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;

@Deprecated
@Disabled
@TeleOp(name="ServoMoveTest")
public class HopperArmTest extends OpMode {


    private OptimizedRobot robot;

    private OptimizedController controller1;


    private Servo servo;

    @Override
    public void init() {
        robot = new OptimizedRobot(gamepad1, gamepad2, telemetry, hardwareMap);

        controller1 = robot.getController1();

        servo = robot.getServo("hopperServo");
    }

    @Override
    public void loop() {

        robot.log("Instructions", "A for one dir and B for the other, have fun");

        if(controller1.getBool(OptimizedController.Key.A)) {
            servo.setPosition(0.8);
        } else if(controller1.getBool(OptimizedController.Key.B)) {
            servo.setPosition(-0.8);
        } else {
            servo.setPosition(0);
        }
    }
}

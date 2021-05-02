package org.firstinspires.ftc.teamcode.main.tests;

import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.internal.Experimental;
import org.firstinspires.ftc.teamcode.internal.OptimizedController;
import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;
import org.firstinspires.ftc.teamcode.main.examples.SampleControllerMapping;
import org.firstinspires.ftc.teamcode.main.examples.SampleHardwareAliasMapping;

import androidx.annotation.RequiresApi;

@Disabled
@Experimental
@TeleOp(name="ExperimentalTeleop", group = "WABOT")
public class ExperimentalTeleop extends OpMode {

    private OptimizedRobot robot;
    private OptimizedController controller1;
    private DcMotor intakeMotor;
    private Servo myServo;


    @Override
    public void init() {
        robot = new OptimizedRobot(gamepad1, gamepad2, telemetry, hardwareMap, new SampleControllerMapping(), new SampleHardwareAliasMapping());
        controller1 = robot.getController1();
        myServo = robot.getServo("myServo");
        intakeMotor = robot.getMotor("intakeMotor", DcMotor.RunMode.RUN_USING_ENCODER, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        // Updating intake, drive, and our autonomous logs
        robot.getDriveFunctions().toggleServoBetweenPos(myServo, 0.0, 0.8, controller1, OptimizedController.Key.A);
        robot.getDriveFunctions().updateTwoWayMotorController("intakeIn", "intakeOut", intakeMotor, controller1);
        robot.updateDrive();
        robot.pushLog();
    }
}

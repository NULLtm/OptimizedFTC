package org.firstinspires.ftc.teamcode.main.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.internal.NumberFunctions;
import org.firstinspires.ftc.teamcode.internal.OptimizedController;
import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;
import org.firstinspires.ftc.teamcode.main.examples.SampleControllerMapping;
import org.firstinspires.ftc.teamcode.main.examples.SampleHardwareAliasMapping;

import java.util.Arrays;

@TeleOp(name="OuttakePowerTuner", group = "WABOT")
public class OuttakePowerTuner extends OpMode {

    private OptimizedRobot robot;

    private OptimizedController controller1;

    private double outtakePower = 0.0;
    private double powerIncrement = 1;

    private DcMotorEx outtakeMotor1;
    private DcMotorEx outtakeMotor2;
    private DcMotor intakeMotor;

    private final double WHEEL_SPINUP_DELAY = 0.5; // In seconds
    private final double HOPPER_SERVO_DELAY = 0.28; // In seconds

    private Servo hopperServo;

    @Override
    public void init() {

        robot = new OptimizedRobot(gamepad1, gamepad2, telemetry, hardwareMap, new SampleControllerMapping(), new SampleHardwareAliasMapping());

        controller1 = robot.getController1();

        outtakeMotor1 = robot.getMotorExByAlias("outtakeMotor1", DcMotor.RunMode.RUN_USING_ENCODER, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeMotor2 = robot.getMotorEx("outtakeMotor2", DcMotor.RunMode.RUN_USING_ENCODER, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);
        hopperServo = robot.getServo("hopperServo", 0.5);
        intakeMotor = robot.getMotor("intakeMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);
        robot.setDriveMode(OptimizedRobot.DriveMode.OMNI);

        robot.log("Status: ", "Robot Initialized.");
    }

    @Override
    public void loop() {

        robot.addLog("Power Increment: ", powerIncrement);
        robot.addLog("Outtake Power: ", outtakePower);


        if(controller1.getOnPress(OptimizedController.Key.DPAD_LEFT)) {
            if(powerIncrement != 1) {
                powerIncrement /= 10;
            } else {
                powerIncrement = 1000;
            }
        } else if(controller1.getOnPress(OptimizedController.Key.DPAD_RIGHT)) {
            if(powerIncrement != 1000) {
                powerIncrement *= 10;
            } else {
                powerIncrement = 1;
            }
        }

        if(controller1.getOnPress(OptimizedController.Key.DPAD_UP) && outtakePower != 1) {
            outtakePower += powerIncrement;
        }
        if(controller1.getOnPress(OptimizedController.Key.DPAD_DOWN) && outtakePower != 0) {
            outtakePower -= powerIncrement;
        }

        if(controller1.getBool(robot.getControl("outtakeAutomatic"))){
            outtakeMotor1.setVelocity(-outtakePower);
            outtakeMotor2.setVelocity(-outtakePower);
            if(robot.synchronousDelayGateOPEN("wheelDelay", getRuntime(), WHEEL_SPINUP_DELAY)){
                robot.addLog("GATE OPEN: ", "FIRST GATE");
                hopperServo.setPosition(0.0);
                if(robot.synchronousDelayGateOPEN("servoDelay", getRuntime(), HOPPER_SERVO_DELAY)){
                    hopperServo.setPosition(0.5);
                    robot.synchronousDelayGateCLOSE("wheelDelay");
                    robot.synchronousDelayGateCLOSE("servoDelay");
                }
            }
        } else {
            robot.synchronousDelayGateCLOSE("wheelDelay");
            robot.synchronousDelayGateCLOSE("servoDelay");
            outtakeMotor1.setVelocity(0);
            outtakeMotor2.setVelocity(0);
            if(controller1.getBool(robot.getControl("hopperManual"))) {hopperServo.setPosition(0.0);} else {hopperServo.setPosition(0.5);}
        }

        robot.getDriveFunctions().updateTwoWayMotorController("intakeIn", "intakeOut", intakeMotor, controller1);
        robot.updateDrive();
    }
}

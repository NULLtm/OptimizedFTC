package org.firstinspires.ftc.teamcode.main.finals;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevSPARKMini;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.internal.OptimizedController;
import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;
import org.firstinspires.ftc.teamcode.internal.RobotConfig;
import org.firstinspires.ftc.teamcode.internal.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.main.examples.SampleControllerMapping;
import org.firstinspires.ftc.teamcode.main.examples.SampleHardwareAliasMapping;

import java.util.Arrays;


@TeleOp(name="FinalTeleop", group = "WABOT")
public class FinalTeleop extends OpMode {

    private OptimizedRobot robot;
    private SampleMecanumDrive drive;
    private OptimizedController controller1;
    private OptimizedController controller2;
    private DcMotorEx outtakeMotor1;
    private DcMotorEx outtakeMotor2;
    private DcMotorEx intakeMotor;
    private Servo hopperServo;
    private Servo wobbleServoTL;
    private Servo wobbleServoTR;
    private Servo wobbleServoBL;
    private Servo wobbleServoBR;
    private DcMotor wobbleMotor;
    private RevSPARKMini sparkMini;

    private boolean runningWobbleUp = false;
    private boolean shooting = false;
    private boolean runningWobbleDown = false;


    private Trajectory builder1;
    private Trajectory builder2;
    private Trajectory builder3;
    private int powerShotStage = 0;

    private final double WHEEL_SPINUP_DELAY = 1; // In seconds
    private final double HOPPER_SERVO_DELAY = 0.34; // In seconds
    private final double WOBBLE_TRAVEL_TIME_UP = 1.7;
    private final double WOBBLE_TRAVEL_TIME_DOWN = 1.5;


    @Override
    public void init() {
        robot = new OptimizedRobot(gamepad1, gamepad2, telemetry, hardwareMap, new SampleControllerMapping(), new SampleHardwareAliasMapping());

        drive = new SampleMecanumDrive(hardwareMap);
        controller1 = robot.getController1();
        controller2 = robot.getController2();

        outtakeMotor1 = robot.getMotorExByAlias("outtakeMotor1", DcMotor.RunMode.RUN_USING_ENCODER, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeMotor2 = robot.getMotorEx("outtakeMotor2", DcMotor.RunMode.RUN_USING_ENCODER, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor = robot.getMotorEx("intakeMotor", DcMotor.RunMode.RUN_USING_ENCODER, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);
        hopperServo = robot.getServo("hopperServo", 0.5);
        wobbleServoTL = robot.getServo("wobbleServoTL", RobotConfig.WOBBLE_GRABBER_OUT.get(0));
        wobbleServoTR = robot.getServo("wobbleServoTR", RobotConfig.WOBBLE_GRABBER_OUT.get(1));
        wobbleServoBL = robot.getServo("wobbleServoBL", RobotConfig.WOBBLE_GRABBER_OUT.get(2));
        wobbleServoBR = robot.getServo("wobbleServoBR", RobotConfig.WOBBLE_GRABBER_OUT.get(3));
        wobbleMotor = robot.getMotorByAlias("wobbleMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);

        builder1 = drive.trajectoryBuilder(new Pose2d(0, -15.0, 0.0))
                .strafeLeft(15.6)
                .build();

        builder2 = drive.trajectoryBuilder(builder1.end())
                .strafeLeft(7.9)
                .build();

        builder3 = drive.trajectoryBuilder(builder2.end())
                .strafeLeft(7.9)
                .build();

        robot.addLog("Status", "Initialized!");
    }

    @Override
    public void loop() {
        robot.addLog("Status", "Running!");


        if(controller1.getBool(OptimizedController.Key.START) || controller2.getBool(OptimizedController.Key.START)) {
            controller1.disableKey(OptimizedController.Key.A, true);
            controller2.disableKey(OptimizedController.Key.B, true);
            controller1.disableKey(OptimizedController.Key.B, true);
            controller2.disableKey(OptimizedController.Key.A, true);
        } else {
            controller1.disableKey(OptimizedController.Key.A, false);
            controller2.disableKey(OptimizedController.Key.B, false);
            controller1.disableKey(OptimizedController.Key.B, false);
            controller2.disableKey(OptimizedController.Key.A, false);
        }

        if(controller2.getBool(robot.getControl("outtakeAutomatic"))){
            double average = (outtakeMotor1.getVelocity() + outtakeMotor2.getVelocity()) / 2.0;
            robot.addLog("AVERAGE SPEED: ", average);
            if(Math.abs(Math.abs(average) - RobotConfig.IDEAL_SHOOTING_LINE_POWER) < 7) {
                hopperServo.setPosition(0.0);
                robot.addLog("RUNNING HOPPER: ", "RUNNING!");
                if(robot.synchronousDelayGateOPEN("servoDelay", getRuntime(), HOPPER_SERVO_DELAY)){
                    hopperServo.setPosition(0.5);
                    robot.synchronousDelayGateCLOSE("servoDelay");
                }
            }
        } else {
            robot.synchronousDelayGateCLOSE("servoDelay");
            if(controller2.getBool(robot.getControl("hopperManual"))) {hopperServo.setPosition(0.0);} else if (controller2.getBool(OptimizedController.Key.X)) {hopperServo.setPosition(1.0);} else {hopperServo.setPosition(0.5);}
        }

        if(controller2.getOnPress(robot.getControl("wobbleUp")) || controller1.getOnPress(robot.getControl("wobbleUp")) || runningWobbleUp) {
            runningWobbleUp = true;
            runningWobbleDown = false;
            robot.synchronousDelayGateCLOSE("wobbleTravelDown");
            wobbleMotor.setPower(0.5);
            if(robot.synchronousDelayGateOPEN("wobbleTravelUp", getRuntime(), WOBBLE_TRAVEL_TIME_UP)) {
                runningWobbleUp = false;
                wobbleMotor.setPower(0);
                robot.synchronousDelayGateCLOSE("wobbleTravelUp");
            }
        }

        if(controller2.getOnPress(robot.getControl("wobbleDown")) || controller1.getOnPress(robot.getControl("wobbleDown")) || runningWobbleDown) {
            runningWobbleDown = true;
            runningWobbleUp = false;
            robot.synchronousDelayGateCLOSE("wobbleTravelUp");
            wobbleMotor.setPower(-0.4);
            if(robot.synchronousDelayGateOPEN("wobbleTravelDown", getRuntime(), WOBBLE_TRAVEL_TIME_DOWN)) {
                runningWobbleDown = false;
                wobbleMotor.setPower(0);
                robot.synchronousDelayGateCLOSE("wobbleTravelDown");
            }
        }

        if(controller2.getBool(OptimizedController.Key.BACK) || controller1.getBool(OptimizedController.Key.BACK)) {
            if(controller2.getBool(robot.getControl("wobbleOpen")) || controller1.getBool(robot.getControl("wobbleOpen"))) {
                wobbleServoTL.setPosition(RobotConfig.WOBBLE_GRABBER_OUT.get(0));
                wobbleServoTR.setPosition(RobotConfig.WOBBLE_GRABBER_OUT.get(1));
            } else if(controller2.getBool(robot.getControl("wobbleClose")) || controller1.getBool(robot.getControl("wobbleClose"))) {
                wobbleServoTL.setPosition(RobotConfig.WOBBLE_GRABBER_IN.get(0));
                wobbleServoTR.setPosition(RobotConfig.WOBBLE_GRABBER_IN.get(1));
            }
        } else {
            if(controller2.getBool(robot.getControl("wobbleOpen")) || controller1.getBool(robot.getControl("wobbleOpen"))) {
                wobbleServoTL.setPosition(RobotConfig.WOBBLE_GRABBER_OUT.get(0));
                wobbleServoTR.setPosition(RobotConfig.WOBBLE_GRABBER_OUT.get(1));
                wobbleServoBL.setPosition(RobotConfig.WOBBLE_GRABBER_OUT.get(2));
                wobbleServoBR.setPosition(RobotConfig.WOBBLE_GRABBER_OUT.get(3));
            } else if(controller2.getBool(robot.getControl("wobbleClose")) || controller1.getBool(robot.getControl("wobbleClose"))) {
                wobbleServoTL.setPosition(RobotConfig.WOBBLE_GRABBER_IN.get(0));
                wobbleServoTR.setPosition(RobotConfig.WOBBLE_GRABBER_IN.get(1));
                wobbleServoBL.setPosition(RobotConfig.WOBBLE_GRABBER_IN.get(2));
                wobbleServoBR.setPosition(RobotConfig.WOBBLE_GRABBER_IN.get(3));
            }
        }

        if(controller2.getBool(OptimizedController.Key.Y)){
            double average = 0;
            switch (powerShotStage) {
                case 0:
                    drive.setPoseEstimate(new Pose2d(0, -15.0, 0.0));
                    outtakeMotor1.setVelocity(RobotConfig.IDEAL_SHOOTING_LINE_POWER - 90);
                    outtakeMotor2.setVelocity(RobotConfig.IDEAL_SHOOTING_LINE_POWER - 90);
                    drive.followTrajectory(builder1);
                    average = (outtakeMotor1.getVelocity() + outtakeMotor2.getVelocity()) / 2.0;
                    while(Math.abs(Math.abs(average) - (RobotConfig.IDEAL_SHOOTING_LINE_POWER - 90)) > 5) {
                        average = (outtakeMotor1.getVelocity() + outtakeMotor2.getVelocity()) / 2.0;
                    }
                    break;
                case 1:
                    hopperServo.setPosition(0.0);
                    time = getRuntime();
                    while(getRuntime() - time < HOPPER_SERVO_DELAY){

                    }
                    hopperServo.setPosition(0.5);
                    drive.turn(Math.toRadians(10));
                    while(Math.abs(Math.abs(average) - (RobotConfig.IDEAL_SHOOTING_LINE_POWER - 90)) > 5) {
                        average = (outtakeMotor1.getVelocity() + outtakeMotor2.getVelocity()) / 2.0;
                    }
                    break;
                case 2:
                    hopperServo.setPosition(0.0);
                    time = getRuntime();
                    while(getRuntime() - time < HOPPER_SERVO_DELAY){

                    }
                    hopperServo.setPosition(0.5);
                    drive.turn(Math.toRadians(6));
                    while(Math.abs(Math.abs(average) - (RobotConfig.IDEAL_SHOOTING_LINE_POWER - 90)) > 5) {
                        average = (outtakeMotor1.getVelocity() + outtakeMotor2.getVelocity()) / 2.0;
                    }
                    break;
                case 3:
                    hopperServo.setPosition(0.0);
                    time = getRuntime();
                    while(getRuntime() - time < HOPPER_SERVO_DELAY){

                    }
                    hopperServo.setPosition(0.5);
                    outtakeMotor1.setPower(0);
                    outtakeMotor2.setPower(0);
                    break;
            }
            powerShotStage++;
        }

        // Updating Intake and Drive Algorithm
        robot.getDriveFunctions().updateTwoWayMotorControllerVelocity("intakeIn", "intakeOut", Arrays.asList(outtakeMotor1, outtakeMotor2), controller2, RobotConfig.IDEAL_SHOOTING_LINE_POWER);
        robot.getDriveFunctions().updateTwoWayMotorController("intakeIn", "intakeOut", intakeMotor, controller1);
        robot.updateDrive(true, true, OptimizedRobot.RobotDirection.BACK, OptimizedRobot.RobotDirection.RIGHT, true);
    }
}

package org.firstinspires.ftc.teamcode.main.beta;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.internal.NumberFunctions;
import org.firstinspires.ftc.teamcode.internal.OptimizedController;
import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;
import org.firstinspires.ftc.teamcode.internal.roadrunner.drive.SampleMecanumDrive;

public class TeleopBeta extends OpMode {

    // Settings
    private final double WHEEL_SPINUP_DELAY = 1; // In seconds
    private final double HOPPER_SERVO_DELAY = 0.35; // In seconds
    private final Vector2d SHOOTING_POSITION = new Vector2d(0.0, 5.0);

    // Main things
    private OptimizedRobot robot;
    private SampleMecanumDrive drive;

    // Controllers
    private OptimizedController controller1;
    private OptimizedController controller2;

    // Main Hardware
    private DcMotor intakeMotor;
    private DcMotor outtakeMotor;
    private Servo hopperServo;

    // Internal crap
    private boolean servoDelay = true;
    private boolean startingDelay = true;
    private double startTime = 0;
    private double servoStartTime = 0;
    private Trajectory trajectory;

    @Override
    public void init() {
        robot = new OptimizedRobot(gamepad1, gamepad2, telemetry, hardwareMap);
        //drive = new SampleMecanumDrive(hardwareMap);


        //robot.initializeOpenCVPipeline(true);

        controller1 = robot.getController1();
        controller2 = robot.getController2();

        //robot.setDriveMode(OptimizedRobot.DriveMode.OMNI);

        intakeMotor = robot.getMotor("intakeMotor");
        outtakeMotor = robot.getMotor("xEncoder"); // Same as our encoder for odometry
        hopperServo = robot.getServo("hopperServo");

        //outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //trajectory = drive.trajectoryBuilder(drive.getPoseEstimate()).splineTo(SHOOTING_POSITION, 0.0).build();

        robot.log("Status ", "Initialized");
    }


    public void loop() {
        robot.addLog("Status ", !controller1.isBeingUsed());
        //robot.addLog("Vision Status ", robot.getVisionOutputToken());

        robot.updateDrive();
        // Intake

        if (controller1.getFloat(OptimizedController.Key.LEFT_TRIGGER) > 0) {
            intakeMotor.setPower(controller1.getFloat(OptimizedController.Key.LEFT_TRIGGER));
        } else {
            intakeMotor.setPower(-controller1.getFloat(OptimizedController.Key.RIGHT_TRIGGER));
        }

        /*if(controller2.getBool(OptimizedController.Key.B)) {
            hopperServo.setPosition(0f);
        } else {
            hopperServo.setPosition(0.5f);
        }*/


        // Outtake
        // This controls a controlled auto-spin up process for outtake
        if (controller2.getBool(OptimizedController.Key.A)) {
            if (startingDelay) {
                outtakeMotor.setPower(-1.0f);
                startTime = getRuntime();
                startingDelay = false;
            }
            if (getRuntime() - startTime > WHEEL_SPINUP_DELAY) {
                // not sure if this val is right
                if (servoDelay) {
                    servoStartTime = getRuntime();
                    servoDelay = false;
                    hopperServo.setPosition(0f);
                }
                if (getRuntime() - servoStartTime > HOPPER_SERVO_DELAY) {
                    hopperServo.setPosition(0.5f);
                    servoDelay = true;
                    startingDelay = true;
                }
            }
        } else {
            hopperServo.setPosition(0.5f);
            servoDelay = true;
            startingDelay = true;
            outtakeMotor.setPower(-controller2.getFloat(OptimizedController.Key.LEFT_TRIGGER));
        }


        // RR Things
            // drive.update();
        /*if(controller1.getBool(OptimizedController.Key.B)) {
            drive.followTrajectory(trajectory);
        }*/


            // Wobble


            // Drive
            //robot.updateDrive();
        }
    }
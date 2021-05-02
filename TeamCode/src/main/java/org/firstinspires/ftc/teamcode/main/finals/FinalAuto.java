package org.firstinspires.ftc.teamcode.main.finals;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.internal.Experimental;
import org.firstinspires.ftc.teamcode.internal.OptimizedController;
import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;
import org.firstinspires.ftc.teamcode.internal.RobotConfig;
import org.firstinspires.ftc.teamcode.internal.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.internal.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.main.examples.SampleControllerMapping;
import org.firstinspires.ftc.teamcode.main.examples.SampleHardwareAliasMapping;
import org.firstinspires.ftc.teamcode.main.pipelines.FinalWABOTPipeline;


@Experimental
@Autonomous(name="FinalAuto")
public class FinalAuto extends LinearOpMode {

    private SampleMecanumDrive drive;
    private OptimizedRobot robot;

    private DcMotor launchMotor;
    private DcMotor wobbleMotor;
    private RevBlinkinLedDriver ledDriver;
    private DcMotorEx outtakeMotor1;
    private Servo hopperServo;
    private Servo wobbleServoTL;
    private Servo wobbleServoTR;
    private Servo wobbleServoBL;
    private Servo wobbleServoBR;
    private DcMotor intakeMotor;

    private DcMotorEx outtakeMotor2;

    private final double POWERSHOT_POWER = RobotConfig.IDEAL_SHOOTING_LINE_POWER - 82;


    private final long HOPPER_SERVO_DELAY = 340; // In seconds

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        robot = new OptimizedRobot(gamepad1, gamepad2, telemetry, hardwareMap, new SampleControllerMapping(), new SampleHardwareAliasMapping());
        robot.initializeOpenCVPipeline(false, new FinalWABOTPipeline());

        ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkDriver");
        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);

        outtakeMotor1 = robot.getMotorExByAlias("outtakeMotor1", DcMotor.RunMode.RUN_USING_ENCODER, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeMotor2 = robot.getMotorEx("outtakeMotor2", DcMotor.RunMode.RUN_USING_ENCODER, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT);
        hopperServo = robot.getServo("hopperServo", 0.5);
        wobbleServoTL = robot.getServo("wobbleServoTL", RobotConfig.WOBBLE_GRABBER_IN.get(0));
        wobbleServoTR = robot.getServo("wobbleServoTR", RobotConfig.WOBBLE_GRABBER_IN.get(1));
        wobbleServoBL = robot.getServo("wobbleServoBL", RobotConfig.WOBBLE_GRABBER_OUT.get(2));
        wobbleServoBR = robot.getServo("wobbleServoBR", RobotConfig.WOBBLE_GRABBER_OUT.get(3));
        wobbleMotor = robot.getMotorByAlias("wobbleMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor = robot.getMotor("intakeMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);

        //wobbleMotor.setPower(0.3);

        // Making sure our localizer thinks we are in the right place
        drive.setPoseEstimate(new Pose2d(-63.0, 15.75, 0.0));

        Pose2d startPose = new Pose2d(-63.0, 15.75, 0.0);

        Trajectory builder1 = drive.trajectoryBuilder(startPose)
                .forward(38.0)
                .build();

        Trajectory builder2 = drive.trajectoryBuilder(builder1.end())
                .splineTo(new Vector2d(0.0, 0.0), 0.0)
                .build();

        Trajectory builder3 = drive.trajectoryBuilder(builder2.end())
                .strafeLeft(7.9)
                .build();

        Trajectory builder4 = drive.trajectoryBuilder(builder3.end())
                .strafeLeft(7.9)
                .build();

        Trajectory WALLbuilder5 = drive.trajectoryBuilder(builder4.end())
                .splineTo(new Vector2d(0.0, 63.0), Math.toRadians(90.0))
                .build();

        Trajectory WALLbuilder6 = drive.trajectoryBuilder(WALLbuilder5.end(), true)
                .splineTo(new Vector2d(-26.0, 36.0), Math.toRadians(180.0))
                .splineTo(new Vector2d(-28, 48.5), Math.toRadians(90.0))
                .build();

        Trajectory WALLbuilder7 = drive.trajectoryBuilder(WALLbuilder6.end())
                .splineTo(new Vector2d(-26.0, 36.0), 0.0)
                .splineTo(new Vector2d(0.0, 36.0), 0.0)
                .splineTo(new Vector2d(-2.0, 55.0), Math.toRadians(90.0))
                .build();

        Trajectory WALLbuilder8 = drive.trajectoryBuilder(WALLbuilder7.end())
                .strafeRight(8.0)
                .build();

        Trajectory MIDbuilder5 = drive.trajectoryBuilder(builder4.end())
                .splineTo(new Vector2d(40.0, 50.0), 0.0)
                .build();

        Trajectory MIDbuilder6 = drive.trajectoryBuilder(MIDbuilder5.end(), true)
                .splineTo(new Vector2d(-26.0, 36.0), Math.toRadians(180.0))
                .splineTo(new Vector2d(-30.4, 48.75), Math.toRadians(90.0))
                .build();

        Trajectory MIDbuilder7 = drive.trajectoryBuilder(MIDbuilder6.end())
                .splineTo(new Vector2d(-26.0, 36.0), 0.0)
                .splineTo(new Vector2d(0.0, 36.0), 0.0)
                .build();

        Trajectory MIDbuilder8 = drive.trajectoryBuilder(MIDbuilder7.end())
                .splineTo(new Vector2d(35.0, 50.0), 0.0)
                .build();

        Trajectory MIDbuilder9 = drive.trajectoryBuilder(MIDbuilder8.end())
                .back(24.0)
                .build();


        Trajectory FARBuilder5 = drive.trajectoryBuilder(builder4.end())
                .splineTo(new Vector2d(47.0,  62.0), Math.toRadians(90.0))
                .build();

        Trajectory FARBuilder6 = drive.trajectoryBuilder(FARBuilder5.end(), true)
                .splineTo(new Vector2d(-24.6, 35.5), Math.toRadians(180.0))
                .build();

        Trajectory FARBuilder7 = drive.trajectoryBuilder(FARBuilder6.end())
                .back(4.0)
                .build();

        Trajectory FARBuilder8 = drive.trajectoryBuilder(FARBuilder7.end())
                .back(2.0)
                .build();

        Trajectory FARBuilder9 = drive.trajectoryBuilder(FARBuilder8.end(), true)
                .splineTo(new Vector2d(-29, 49), Math.toRadians(90.0))
                .build();

        Trajectory FARBuilder10 = drive.trajectoryBuilder(FARBuilder9.end())
                .splineTo(new Vector2d(-26.0, 36.0), 0.0)
                .splineTo(new Vector2d(0.0, 36.0), 0.0)
                .build();

        Trajectory FARBuilder11 = drive.trajectoryBuilder(FARBuilder10.end())
                .splineTo(new Vector2d(47.0, 58.0), Math.toRadians(90.0))
                .build();

        Trajectory FARBuilder12 = drive.trajectoryBuilder(FARBuilder11.end())
                .strafeLeft(38.0)
                .build();

        robot.log("Status: ", "Ready to go!");

        waitForStart();

        String output = "";


        drive.followTrajectory(builder1);
        output = robot.getVisionOutputToken();
        outtakeMotor1.setVelocity(POWERSHOT_POWER);
        outtakeMotor2.setVelocity(POWERSHOT_POWER);
        drive.followTrajectory(builder2);
        shootRing();
        drive.turn(Math.toRadians(10));
        shootRing();
        drive.turn(Math.toRadians(7));
        shootRing();
        outtakeMotor1.setPower(0);
        outtakeMotor2.setPower(0);
        if(output.equals("0")){
            drive.followTrajectory(WALLbuilder5);
            dropWobble();
            drive.followTrajectory(WALLbuilder6);
            pickUpWobble();
            drive.followTrajectory(WALLbuilder7);
            dropWobble();
            drive.followTrajectory(WALLbuilder8);
            wobbleMotor.setPower(1.0);
            sleep(400);
            wobbleMotor.setPower(0);
        } else if(output.equals("1")) {
            drive.followTrajectory(MIDbuilder5);
            intakeMotor.setPower(-1);
            dropWobble();
            drive.followTrajectory(MIDbuilder6);
            pickUpWobble();
            drive.followTrajectory(MIDbuilder7);
            intakeMotor.setPower(1);
            outtakeMotor1.setVelocity(RobotConfig.IDEAL_SHOOTING_LINE_POWER);
            outtakeMotor2.setVelocity(RobotConfig.IDEAL_SHOOTING_LINE_POWER);
            sleep(700);
            hopperServo.setPosition(0);
            sleep(900);
            hopperServo.setPosition(0.5);
            intakeMotor.setPower(0.0);
            drive.followTrajectory(MIDbuilder8);
            dropWobble();
            drive.followTrajectory(MIDbuilder9);

        } else {
            drive.followTrajectory(FARBuilder5);
            intakeMotor.setPower(-1);
            dropWobble();
            drive.followTrajectory(FARBuilder6);
            sleep(500);
            intakeMotor.setPower(1);
            sleep(400);
            intakeMotor.setPower(-1);
            drive.followTrajectory(FARBuilder7);
            intakeMotor.setPower(1);
            sleep(400);
            intakeMotor.setPower(-1);
            sleep(200);
            drive.followTrajectory(FARBuilder8);
            drive.followTrajectory(FARBuilder9);
            pickUpWobble();
            drive.followTrajectory(FARBuilder10);
            intakeMotor.setPower(1);
            outtakeMotor1.setVelocity(RobotConfig.IDEAL_SHOOTING_LINE_POWER);
            outtakeMotor2.setVelocity(RobotConfig.IDEAL_SHOOTING_LINE_POWER);
            sleep(700);
            hopperServo.setPosition(0);
            sleep(1500);
            hopperServo.setPosition(0.5);
            intakeMotor.setPower(0.0);
            drive.followTrajectory(FARBuilder11);
            dropWobble();
            drive.followTrajectory(FARBuilder12);
        }
    }

    private void shootRing(){
        double average = (outtakeMotor1.getVelocity() + outtakeMotor2.getVelocity()) / 2.0;
        while(Math.abs(Math.abs(average) - POWERSHOT_POWER) > 5) {
            average = (outtakeMotor1.getVelocity() + outtakeMotor2.getVelocity()) / 2.0;
            robot.addLog("outtake", average);
            robot.addLog("one", outtakeMotor1.getVelocity());
            robot.addLog("two", outtakeMotor2.getVelocity());
            robot.pushLog();
        }
        hopperServo.setPosition(0.0);
        sleep(HOPPER_SERVO_DELAY);
        hopperServo.setPosition(0.5);
    }

    private void dropWobble() {
        wobbleServoTL.setPosition(RobotConfig.WOBBLE_GRABBER_OUT.get(0));
        wobbleServoTR.setPosition(RobotConfig.WOBBLE_GRABBER_OUT.get(1));
        wobbleServoBL.setPosition(RobotConfig.WOBBLE_GRABBER_OUT.get(2));
        wobbleServoBR.setPosition(RobotConfig.WOBBLE_GRABBER_OUT.get(3));
        wobbleMotor.setPower(-0.8);
        sleep(500);
        wobbleMotor.setPower(0);
    }

    private void pickUpWobble() {
        //wobbleServo.setPosition(0.5);
        wobbleMotor.setPower(-0.8);
        sleep(700);
        wobbleMotor.setPower(0);
        wobbleServoTL.setPosition(RobotConfig.WOBBLE_GRABBER_IN.get(0));
        wobbleServoTR.setPosition(RobotConfig.WOBBLE_GRABBER_IN.get(1));
        wobbleServoBL.setPosition(RobotConfig.WOBBLE_GRABBER_IN.get(2));
        wobbleServoBR.setPosition(RobotConfig.WOBBLE_GRABBER_IN.get(3));
        sleep(500);
        wobbleMotor.setPower(1.0);
        sleep(600);
        wobbleMotor.setPower(0.0);
    }
}

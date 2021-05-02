package org.firstinspires.ftc.teamcode.main.beta;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.internal.Experimental;
import org.firstinspires.ftc.teamcode.internal.OptimizedController;
import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;
import org.firstinspires.ftc.teamcode.internal.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.main.examples.SampleControllerMapping;
import org.firstinspires.ftc.teamcode.main.examples.SampleHardwareAliasMapping;
import org.firstinspires.ftc.teamcode.main.pipelines.FinalWABOTPipeline;


@Experimental
@Autonomous(name="VisionAuto")
public class VisionAuto extends LinearOpMode {

    private SampleMecanumDrive drive;
    private OptimizedRobot robot;

    private DcMotor launchMotor;
    private DcMotor wobbleMotor;
    private RevBlinkinLedDriver ledDriver;
    private DcMotor outtakeMotor1;
    private Servo hopperServo;
    private Servo wobbleServo;
    private DcMotor intakeMotor;

    private DcMotor outtakeMotor2;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        robot = new OptimizedRobot(gamepad1, gamepad2, telemetry, hardwareMap, new SampleControllerMapping(), new SampleHardwareAliasMapping());
        robot.initializeOpenCVPipeline(false, new FinalWABOTPipeline());

        ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkDriver");
        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);

        outtakeMotor1 = robot.getMotorByAlias("outtakeMotor1", DcMotor.RunMode.RUN_USING_ENCODER, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeMotor2 = robot.getMotor("outtakeMotor2", DcMotor.RunMode.RUN_USING_ENCODER, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);
        hopperServo = robot.getServo("hopperServo", 0.5);
        wobbleServo = robot.getServo("wobbleServo", 0.0);
        wobbleMotor = robot.getMotorByAlias("wobbleMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor = robot.getMotor("intakeMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);

        wobbleMotor.setPower(0.3);

        // Making sure our localizer thinks we are in the right place
        drive.setPoseEstimate(new Pose2d(-63.0, 15.75, 0.0));

        Pose2d startPose = new Pose2d(-63.0, 15.75, 0.0);

        Trajectory builder1 = drive.trajectoryBuilder(startPose)
                .forward(38.0)
                .build();

        robot.log("Status: ", "Ready to go!");

        waitForStart();

        String output = "";

        drive.followTrajectory(builder1);
        sleep(700);
        output = robot.getVisionOutputToken();
    }
}

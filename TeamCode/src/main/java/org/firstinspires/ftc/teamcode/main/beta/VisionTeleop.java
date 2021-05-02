package org.firstinspires.ftc.teamcode.main.beta;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.internal.Experimental;
import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;
import org.firstinspires.ftc.teamcode.internal.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.main.pipelines.TestingWABOTPipeline;


@Experimental
@TeleOp(name="VisionTeleop")
public class VisionTeleop extends OpMode {

    private SampleMecanumDrive drive;
    private OptimizedRobot robot;

    private DcMotor launchMotor;
    private DcMotor wobbleMotor;
    private RevBlinkinLedDriver ledDriver;

    @Override
    public void init() {
        robot = new OptimizedRobot(gamepad1, gamepad2, telemetry, hardwareMap);

        robot.initializeOpenCVPipeline(true, new TestingWABOTPipeline());
        //robot.initializeDriveMotors(false);

        ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkDriver");
        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);

        robot.log("Status: ", "Initialized!");
    }

    @Override
    public void loop() {
        robot.log("VISION OUTPUT", robot.getVisionOutputToken());
    }
}

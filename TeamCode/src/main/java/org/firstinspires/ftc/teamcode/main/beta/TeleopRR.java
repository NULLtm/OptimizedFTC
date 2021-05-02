package org.firstinspires.ftc.teamcode.main.beta;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.internal.OptimizedController;
import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;
import org.firstinspires.ftc.teamcode.internal.roadrunner.drive.SampleMecanumDrive;

public class TeleopRR extends OpMode {
    private OptimizedRobot robot;

    private OptimizedController controller1;

    private DcMotor intakeMotor;

    SampleMecanumDrive drive;

    Vector2d shootPos = new Vector2d(0.0,5.0);

    @Override
    public void init() {
        robot = new OptimizedRobot(gamepad1, gamepad2, telemetry, hardwareMap);

        robot.initializeRoadRunner();

        drive = robot.getInternalRR();

        // Grabbing controller
        controller1 = robot.getController1();

        // A sample intake motor
        intakeMotor = robot.getMotor("intakeMotor");

        // Stuff
        robot.setDriveMode(OptimizedRobot.DriveMode.OMNI);

        robot.log("Status ", "Initialized");
    }
    @Override
    public void loop() {
        Pose2d poseEstimate = robot.getRRPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();
        if(controller1.getBool(OptimizedController.Key.A)) {
            drive.trajectoryBuilder(drive.getPoseEstimate())
                    .splineTo(shootPos, 0.0);
        }
    }
}

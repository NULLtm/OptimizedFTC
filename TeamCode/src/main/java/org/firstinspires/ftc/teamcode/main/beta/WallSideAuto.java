package org.firstinspires.ftc.teamcode.main.beta;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;
import org.firstinspires.ftc.teamcode.internal.roadrunner.drive.SampleMecanumDrive;

public class WallSideAuto extends LinearOpMode {

    private SampleMecanumDrive drive;
    private OptimizedRobot robot;

    private DcMotor launchMotor;
    private DcMotor wobbleMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        robot = new OptimizedRobot(gamepad1, gamepad2, telemetry, hardwareMap);

        // Making sure our localizer thinks we are in the right place
        drive.setPoseEstimate(new Pose2d(-63.0, 15.0, 0.0));

        Pose2d startPose = new Pose2d(-63.0, 15.0, 0.0);

        Trajectory builder1 = drive.trajectoryBuilder(startPose)
                .forward(39.0)
                .splineTo(new Vector2d(0.0, 5.0), 0.0)
                .build();

        Trajectory builder2 = drive.trajectoryBuilder(builder1.end())
                .strafeLeft(8.0)
                .build();

        Trajectory builder3 = drive.trajectoryBuilder(builder2.end())
                .strafeLeft(8.0)
                .build();

        Trajectory builder4 = drive.trajectoryBuilder(builder3.end())
                .strafeLeft(41.0)
                .build();

        Trajectory builder5 = drive.trajectoryBuilder(builder4.end())
                .forward(14.0)
                .build();

        Trajectory builder6 = drive.trajectoryBuilder(builder5.end(), true)
                .splineTo(new Vector2d(-26.0, 36.0), Math.toRadians(180.0))
                .splineTo(new Vector2d(-38.0, 50.0), Math.toRadians(90.0))
                .build();

        Trajectory builder7 = drive.trajectoryBuilder(builder6.end())
                .splineTo(new Vector2d(-26.0, 36.0), 0.0)
                .splineTo(new Vector2d(0.0, 36.0), 0.0)
                .build();

        Trajectory builder8 = drive.trajectoryBuilder(builder7.end())
                .strafeLeft(26.0)
                .build();

        Trajectory builder9 = drive.trajectoryBuilder(builder8.end())
                .forward(10.0)
                .build();

        robot.log("Status: ", "Ready to go!");

        waitForStart();
      

        drive.followTrajectory(builder1);
        drive.followTrajectory(builder2);
        drive.followTrajectory(builder3);
        drive.followTrajectory(builder4);
        drive.followTrajectory(builder5);
        drive.followTrajectory(builder6);
        drive.followTrajectory(builder7);
        drive.followTrajectory(builder8);
        drive.followTrajectory(builder9);
    }
}

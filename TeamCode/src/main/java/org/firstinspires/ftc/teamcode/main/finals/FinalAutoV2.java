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
import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;
import org.firstinspires.ftc.teamcode.internal.RobotConfig;
import org.firstinspires.ftc.teamcode.internal.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.main.examples.SampleControllerMapping;
import org.firstinspires.ftc.teamcode.main.examples.SampleHardwareAliasMapping;
import org.firstinspires.ftc.teamcode.main.pipelines.FinalWABOTPipeline;


@Experimental
@Autonomous(name="FinalAuto")
public class FinalAutoV2 extends LinearOpMode {

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
                .forward(25.0)
                .build();

        /** WALL SIDE **/
        Trajectory WALLbuilder1 = drive.trajectoryBuilder(builder2.end())
                .lineToLinearHeading(new Pose2d(0.0, 62.0, Math.toRadians(90.0)))
                .build();

        Trajectory WALLbuilder2 = drive.trajectoryBuilder(WALLbuilder1.end())
                .addDisplacementMarker(40.0, () -> {wobbleMotor.setPower(0);})
                .lineToLinearHeading(new Pose2d(-34.0, 50.0, Math.toRadians(270.0)))
                .build();

        Trajectory WALLbuilder3 = drive.trajectoryBuilder(WALLbuilder2.end())
                .lineToLinearHeading(new Pose2d(0.0, 55.0, Math.toRadians(90.0)))
                .build();

        Trajectory WALLbuilder4 = drive.trajectoryBuilder(WALLbuilder3.end())
                .strafeRight(13.0)
                .build();


        /** MID SIDE **/

        Trajectory MIDbuilder1 = drive.trajectoryBuilder(builder2.end())
                .lineToLinearHeading(new Pose2d(40.0, 47.0, 0.0))
                .build();

        Trajectory MIDbuilder2 = drive.trajectoryBuilder(MIDbuilder1.end())
                .lineToLinearHeading(new Pose2d(-12.0, 36.0, 0.0))
                .build();

        Trajectory MIDbuilder3 = drive.trajectoryBuilder(MIDbuilder2.end(), true)
                .splineTo(new Vector2d(-34.0, 50.0), Math.toRadians(90.0))
                .build();

        Trajectory MIDbuilder4 = drive.trajectoryBuilder(MIDbuilder3.end())
                .splineTo(new Vector2d(-12.0, 36.0), 0.0)
                .forward(12.0)
                .build();

        Trajectory MIDbuilder5 = drive.trajectoryBuilder(MIDbuilder4.end())
                .lineToLinearHeading(new Pose2d(33.0, 47.0, 0.0))
                .build();

        Trajectory MIDbuilder6 = drive.trajectoryBuilder(MIDbuilder5.end())
                .back(25.0)
                .build();

        /** FAR SIDE **/

        Trajectory FARBuilder1 = drive.trajectoryBuilder(builder2.end())
                .lineToLinearHeading(new Pose2d(63.0, 36.0, Math.toRadians(180.0)))
                .build();

        Trajectory FARBuilder2 = drive.trajectoryBuilder(FARBuilder1.end())
                .lineToLinearHeading(new Pose2d(-12.0, 36.0, 0.0))
                .build();

        Trajectory FARBuilder3 = drive.trajectoryBuilder(FARBuilder2.end())
                .back(2.0)
                .build();

        Trajectory FARBuilder4 = drive.trajectoryBuilder(FARBuilder3.end())
                .back(2.0)
                .build();

        Trajectory FARBuilder5 = drive.trajectoryBuilder(FARBuilder4.end())
                .back(2.0)
                .build();

        Trajectory FARBuilder6 = drive.trajectoryBuilder(FARBuilder5.end(), true)
                .splineTo(new Vector2d(-34.0, 50.0), Math.toRadians(90.0))
                .build();

        Trajectory FARBuilder7 = drive.trajectoryBuilder(FARBuilder6.end())
                .splineTo(new Vector2d(-18.0, 36.0), 0.0)
                .forward(18.0)
                .build();

        Trajectory FARBuilder8 = drive.trajectoryBuilder(FARBuilder7.end())
                .lineToLinearHeading(new Pose2d(58.0, 36.0, Math.toRadians(180.0)))
                .build();

        Trajectory FARBuilder9 = drive.trajectoryBuilder(FARBuilder8.end())
                .forward(47.0)
                .build();

        robot.autonomousLog("Status: ", "Ready to go!");
        robot.pushLog();

        waitForStart();

        String output = "";


        drive.followTrajectory(builder1);
        output = robot.getVisionOutputToken();
        outtakeMotor1.setVelocity(POWERSHOT_POWER);
        outtakeMotor2.setVelocity(POWERSHOT_POWER);
        drive.followTrajectory(builder2);
        shootRing();
        drive.turn(Math.toRadians(7));
        shootRing();
        drive.turn(-Math.toRadians(7));
        shootRing();
        outtakeMotor1.setPower(0);
        outtakeMotor2.setPower(0);
        if(output.equals("0")){
            drive.followTrajectory(WALLbuilder1);
            dropWobble();
            drive.followTrajectory(WALLbuilder2);
            pickUpWobble();
            drive.followTrajectory(WALLbuilder3);
            dropWobble();
            drive.followTrajectory(WALLbuilder4);
            wobbleMotor.setPower(1.0);
            sleep(400);
            wobbleMotor.setPower(0);
        } else if(output.equals("1")) {
            drive.followTrajectory(MIDbuilder1);
            intakeMotor.setPower(-1);
            dropWobble();
            drive.followTrajectory(MIDbuilder2);
            drive.followTrajectory(MIDbuilder3);
            pickUpWobble();
            drive.followTrajectory(MIDbuilder4);
            intakeMotor.setPower(1);
            outtakeMotor1.setVelocity(RobotConfig.IDEAL_SHOOTING_LINE_POWER);
            outtakeMotor2.setVelocity(RobotConfig.IDEAL_SHOOTING_LINE_POWER);
            sleep(700);
            hopperServo.setPosition(0);
            sleep(900);
            hopperServo.setPosition(0.5);
            intakeMotor.setPower(0.0);
            drive.followTrajectory(MIDbuilder5);
            dropWobble();
            drive.followTrajectory(MIDbuilder6);

        } else {
            drive.followTrajectory(FARBuilder1);
            intakeMotor.setPower(-1);
            dropWobble();
            drive.followTrajectory(FARBuilder2);
            sleep(500);
            intakeMotor.setPower(1);
            sleep(400);
            intakeMotor.setPower(-1);
            drive.followTrajectory(FARBuilder3);
            intakeMotor.setPower(1);
            sleep(400);
            intakeMotor.setPower(-1);
            sleep(200);
            drive.followTrajectory(FARBuilder4);
            drive.followTrajectory(FARBuilder5);
            pickUpWobble();
            drive.followTrajectory(FARBuilder6);
            intakeMotor.setPower(1);
            outtakeMotor1.setVelocity(RobotConfig.IDEAL_SHOOTING_LINE_POWER);
            outtakeMotor2.setVelocity(RobotConfig.IDEAL_SHOOTING_LINE_POWER);
            sleep(700);
            hopperServo.setPosition(0);
            sleep(1500);
            hopperServo.setPosition(0.5);
            intakeMotor.setPower(0.0);
            drive.followTrajectory(FARBuilder7);
            dropWobble();
            drive.followTrajectory(FARBuilder8);
        }
    }

    private void shootRing(){
        double average = (outtakeMotor1.getVelocity() + outtakeMotor2.getVelocity()) / 2.0;
        while(Math.abs(Math.abs(average) - POWERSHOT_POWER) > 5) {
            average = (outtakeMotor1.getVelocity() + outtakeMotor2.getVelocity()) / 2.0;
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

package org.firstinspires.ftc.teamcode.internal;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import java.util.Arrays;
import java.util.List;


/**
 * This is the OFFICIAL robot settings file: for anything you need to change, you'll probably come here.
 * This file holds details about hardware names and about certain measurements on the robot that will need to be tuned each year.
 *
 * TODO: READ VVVV
 * This file is used for attributes that affect EVERY op mode! If you want to add/remove attributes, keep this in mind!!
 *
 * @author Owen_Boseley
 *
 *
 * TODO: READ THIS vvvv
 * NOTE: The Standard Convention for Motor placement is FL, FR, BL ,BR
 * NOTE: The standard naming convention has changed from FLMotor to frontLeftMotor.
 * TODO: READ THIS ^^^^
 */
public class RobotConfig {


    // TODO: Please add your desired motor directions into this array in the form {FL, FR, BL ,BR}.
    protected static final DcMotorSimple.Direction[] motorDirections = {Direction.REVERSE, Direction.FORWARD, Direction.REVERSE, Direction.FORWARD};

    // TODO: Please enter the radius of the wheels in INCHES.
    protected static final double WHEEL_RADIUS = 5.15;

    // FROM ULTIMATE GOAL -- DELETE IF YOU WANT
    public static final double IDEAL_SHOOTING_LINE_POWER = 1720;

    // TODO: Please enter the number of ticks per revolution for your encoders (wheel-motor encoders, NOT odometer encoders).
    protected static final double ENCODER_TICK_PER_REV = 1680;

    // TODO: If you have a webcam, please enter here. If not, set this variable to null.
    protected static final String WEBCAM_NAME = "Webcam 1";

    // Used in 2021 Ultimate Goal for Louis to be able to override driver 1 if held down
    protected static final OptimizedController.Key NUCLEAR_KEY = OptimizedController.Key.RIGHT_BUMPER;

    public static final List<Double> WOBBLE_GRABBER_IN = Arrays.asList(0.45, 0.5, 0.0, 0.5);

    public static final List<Double> WOBBLE_GRABBER_OUT = Arrays.asList(0.0, 1.0, 0.6, 0.15);


}

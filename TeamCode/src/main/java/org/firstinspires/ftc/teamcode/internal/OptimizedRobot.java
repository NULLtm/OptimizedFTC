package org.firstinspires.ftc.teamcode.internal;

import android.os.Build;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.internal.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

import java.util.HashMap;

import androidx.annotation.RequiresApi;

/**
 * This class is a "repackaged" version
 * of standard FTC Programming using OpMode/LinearOpMode classes.
 * Not only does it strive to help beginner programmers, but it also adds optimizations in the controllers and other things.
 *
 * @author Owen Boseley - Class of 2021
 */
@Repackaged
public class OptimizedRobot {

    /**
     * Robot direction state for drive train algorithm
     */
    public enum RobotDirection {
        FRONT, BACK, LEFT, RIGHT;
    }

    /**
     * Our internal gampads
     */
    private Gamepad gamepad1 = null, gamepad2 = null;

    /**
     * The virtual controllers
     */
    private List<OptimizedController> controllers = new ArrayList<>();

    /**
     * The current drive mode of the robot. More info below here: {@link #getDriveMode()}
     */
    private DriveMode driveMode = DriveMode.STOPPED;

    /**
     * The drive functions class mentioned below
     */
    private OptimizedDriveFunctions functions = null;

    /**
     * Stores the telemetry instance from {@link com.qualcomm.robotcore.eventloop.opmode.OpMode}
     * Used for outputting data to the drive station console.
     */
    private Telemetry telemetry = null;

    /**
     * Stores an instance of our OpenCV Loader, which should never need to be edited unless for version changes
     */
    private OpenCVLoader loader = null;

    // Ignore these, these are internal stuff
    private boolean hasInitializedMotors = false;
    private boolean hasUpdatedDrive = false;

    /**
     * Holds the info for custom delays
     */
    private HashMap<String, Boolean> delayInfoBools = new HashMap<String, Boolean>();
    private HashMap<String, Double> delayInfoTimes = new HashMap<String, Double>();

    /**
     * Stores 4 booleans for if any of the drive train motors are disabled for testing purposes
     */
    private boolean[] disabledMotors = {false, false, false, false};


    /**
     * Our map of aliases of hardware components
     */
    private HashMap<String, List<String>> aliasMap = null;

    /**
     * This holds a custom set of controls for Teleop to use
     */
    @Experimental
    private HashMap<String, OptimizedController.Key> controlMap = null;

    /**
     * Stores the current robot status
     */
    private RobotStatus status = RobotStatus.INITIALIZING;


    /**
     * Our mecanum drive for use in roadrunner
     */
    private SampleMecanumDrive mecanumDrive = null;


    /**
     * An array to store the drive train motors. Stored in the format: FL, FR, BL, BR.
     * P.S. All of the standards in this code are written out here: {@link RobotConfig}
     */
    protected DcMotor[] motors = new DcMotor[4];

    /**
     * Stores the hardwareMap from the phone. This is where we get all of our hardware components for usage in OpModes
     */
    private HardwareMap internalMap = null;

    // Just some conversion ratios, ignore these
    private final double CM_PER_INCH = 2.56;
    private final double CM_PER_FOOT = 30.48;

    /**
     * The Wheel Radius of the main drive train wheels in CM
     */
    private final double WHEEL_RADIUS = RobotConfig.WHEEL_RADIUS * CM_PER_INCH;

    /**
     * The circumference of the drive train wheels in CM
     */
    protected final double CIRCUMFERENCE = Math.PI * WHEEL_RADIUS * 2;

    /**
     * An Enum representing the possible robot statuses
     */
    private enum RobotStatus {
        INITIALIZING, READY, IDLE, STOPPED, DRIVING
    }

    /**
     * An Enum representing the possible drive modes
     */
    public enum DriveMode {
        OMNI, TANK, STOPPED
    }

    /**
     * Sets up a virtual controller with a given start key (START + KEY to activate)
     * @param requiredGamepad If you want a required gamepad (1 or 2) to use this, then tada!
     * @param startKey The START + this key to activate this virtual controller
     * @return The controller obj
     */
    public OptimizedController setUpVirtualController(Gamepad requiredGamepad, OptimizedController.Key startKey) {
        OptimizedController o = new OptimizedController(requiredGamepad, startKey, gamepad1, gamepad2);
        controllers.add(o);
        return o;
    }

    /**
     * This method will iterate through our virtual controllers looking for when a gamepad hits the correct START + startKey combo to activate the controller
     * This method is pertinent to correct teleop functionality!!!
     */
    public void updateVirtualControllerStartup() {
        for(OptimizedController controller : controllers) {
            controller.updateInternalGamepad();
        }
    }

    /**
     * Constructor oi this class
     * @param gamepad1 The gamepad1 var inherited from OpMode/LinearOpMode
     * @param gamepad2 The gamepad2 var inherited from OpMode/LinearOpMode
     * @param telemetry The telemetry var inherited from OpMode/LinearOpMode
     * @param hardwareMap The hardwareMap var inherited from OpMode/LinearOpMode
     */
    public OptimizedRobot(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        functions = new OptimizedDriveFunctions(this);

        internalMap = hardwareMap;
        this.telemetry = telemetry;

        status = RobotStatus.READY;
    }

    /**
     * Constructor oi this class
     * @param gamepad1 The gamepad1 var inherited from OpMode/LinearOpMode
     * @param gamepad2 The gamepad2 var inherited from OpMode/LinearOpMode
     * @param telemetry The telemetry var inherited from OpMode/LinearOpMode
     * @param hardwareMap The hardwareMap var inherited from OpMode/LinearOpMode
     * @param controlMap A hashmap of string names to keys for teleop -- might be useless, idk
     */
    public OptimizedRobot(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap, ControllerMapping controlMap){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        functions = new OptimizedDriveFunctions(this);
        this.controlMap = controlMap.initializeMapping(new HashMap<String, OptimizedController.Key>());

        internalMap = hardwareMap;
        this.telemetry = telemetry;

        status = RobotStatus.READY;
    }

    /**
     * Constructor oi this class
     * @param gamepad1 The gamepad1 var inherited from OpMode/LinearOpMode
     * @param gamepad2 The gamepad2 var inherited from OpMode/LinearOpMode
     * @param telemetry The telemetry var inherited from OpMode/LinearOpMode
     * @param hardwareMap The hardwareMap var inherited from OpMode/LinearOpMode
     * @param hardwareMapping A hashmap of hardware map names to aliases for use in op modes (to avoid to confusion)
     */
    public OptimizedRobot(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap, HardwareAliasMapping hardwareMapping){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        functions = new OptimizedDriveFunctions(this);
        this.aliasMap = hardwareMapping.initializeMapping(new HashMap<String, List<String>>());

        internalMap = hardwareMap;
        this.telemetry = telemetry;

        status = RobotStatus.READY;
    }

    /**
     * Constructor oi this class
     * @param gamepad1 The gamepad1 var inherited from OpMode/LinearOpMode
     * @param gamepad2 The gamepad2 var inherited from OpMode/LinearOpMode
     * @param telemetry The telemetry var inherited from OpMode/LinearOpMode
     * @param hardwareMap The hardwareMap var inherited from OpMode/LinearOpMode
     * @param controlMap A hashmap of string names to keys for teleop -- might be useless, idk
     * @param hardwareMapping A hashmap of hardware map names to aliases for use in op modes (to avoid to confusion)
     */
    public OptimizedRobot(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap, ControllerMapping controlMap, HardwareAliasMapping hardwareMapping){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        functions = new OptimizedDriveFunctions(this);
        this.controlMap = controlMap.initializeMapping(new HashMap<String, OptimizedController.Key>());
        this.aliasMap = hardwareMapping.initializeMapping(new HashMap<String, List<String>>());

        internalMap = hardwareMap;
        this.telemetry = telemetry;

        status = RobotStatus.READY;
    }

    /**
     * Used to initialize OpenCV and the camera for usage
     * @usage Only call this method during initialization (I mean, you can call it after but like... why?)
     * @param activateCam Whether or not to display the camera output on the phone
     */
    public void initializeOpenCVPipeline(boolean activateCam, OptimizedOpenCVPipeline pipeline){
        status = RobotStatus.INITIALIZING;
        loader = new OpenCVLoader(internalMap, activateCam, pipeline);

        status = RobotStatus.READY;
    }

    /**
     * Initializes our roadrunner shtuff
     */
    public void initializeRoadRunner() {
        mecanumDrive = new SampleMecanumDrive(internalMap);
    }

    /**
     * Used if you do NOT want to use the repackaged methods in this class -- up to you
     * @return The mecanum drive obj used for RR processes
     */
    public SampleMecanumDrive getInternalRR() {
        return mecanumDrive;
    }

    /**
     * Returns position estimate for road runner
     * @return
     */
    @Repackaged
    public Pose2d getRRPoseEstimate() {
        return mecanumDrive.getPoseEstimate();
    }

    /**
     * Used to follow a given trajectory using RR
     * @param trajectory the trajectory BUILDER (without the .build() at the end) to follow
     */
    @Repackaged
    public void followRRTrajectory(TrajectoryBuilder trajectory) {
        mecanumDrive.followTrajectory(trajectory.build());
    }

    /**
     * Useful delay method for teleop opmodes -- want to delay a piece of code without stopping the thread entirely? Use THIS!
     * NOTE: This method will only OPEN the gate after the delay, and will not close it. You'll need to call {@link #synchronousDelayGateCLOSE(String)} to close it!
     * NOTE: Use
     * @param delayName The delay name (can be anything you want)
     * @param runtime The runtime var inherited from {@link com.qualcomm.robotcore.eventloop.opmode.OpMode}
     * @param delayInSeconds The delay for this gate
     * @return Returns whether or not to let this gate through (true or false)
     */
    @Experimental
    public boolean synchronousDelayGateOPEN(String delayName, double runtime, double delayInSeconds) {
        if(delayInfoBools.get(delayName) == null) {
            delayInfoBools.put(delayName, false);
        }

        if(!delayInfoBools.get(delayName)) {
            if(delayInfoTimes.get(delayName) == null)
                delayInfoTimes.put(delayName, runtime);
            else
                delayInfoTimes.put(delayName, runtime);
            delayInfoBools.put(delayName, true);
        } else if(runtime - delayInfoTimes.get(delayName) >= delayInSeconds) {
            return true;
        }

        return false;
    }

    /**
     * Closes a delay gate
     * @param delayName The name of the gate to close
     */
    @Experimental
    public void synchronousDelayGateCLOSE(String delayName) {
        delayInfoBools.put(delayName, false);
    }

    /**
     * Useful delay method for teleop opmodes -- want to delay a piece of code without stopping the thread entirely? Use THIS!
     * NOTE: This method will OPEN the gate for one update before closing it again
     * @param delayName The delay name (can be anything you want)
     * @param runtime The runtime var inherited from {@link com.qualcomm.robotcore.eventloop.opmode.OpMode}
     * @param delayInSeconds The delay for this gate
     * @return Returns whether or not to let this gate through (true or false)
     */
    @Experimental
    public boolean synchronousDelayGateCOMPLETE(String delayName, double runtime, double delayInSeconds) {
        if(delayInfoBools.get(delayName) == null) {
            delayInfoBools.put(delayName, false);
        }

        if(!delayInfoBools.get(delayName)) {
            if(delayInfoTimes.get(delayName) == null)
                delayInfoTimes.put(delayName, runtime);
            else
                delayInfoTimes.put(delayName, runtime);
            delayInfoBools.put(delayName, true);
        } else if(runtime - delayInfoTimes.get(delayName) >= delayInSeconds) {
            delayInfoTimes.put(delayName, runtime);
            return true;
        } else {
            delayInfoBools.put(delayName, false);
        }

        return false;
    }

    /**
     * Used to grab our automated controls
     * @param controlName The name of the control specified in your mapping
     * @return The key associated with the control
     */
    @Experimental
    public OptimizedController.Key getControl(String controlName) {
        return controlMap.get(controlName);
    }

    /**
     * Returns the data returned from {@link OptimizedOpenCVPipeline}
     * @usage Only call this method after calling initialize on the loader
     * @return The data
     */
    public String getVisionOutputToken() {
        return loader.pipeline.getVisionOutput();
    }

    /**
     * Add a log to the register--won't show up on the phone until the {@link #pushLog()} method is called
     * @param title The title of the log
     * @param value The value of data you want to log
     */
    public void addLog(String title, Object value){
        telemetry.addData("WABOT: "+title, value);
    }

    /**
     * Push the autonomous logs, overriding any old messages with new ones
     * DO NOT USE during teleop (unless you know what you are doing)
     */
    public void pushLog() {
        telemetry.update();
    }

    /**
     * Our main driving method: uses one of a couple of drive algorithms to calculate and assign motor powers
     * @usage Only call this method after initialization and instantiating the robot
     * This algorithm makes gamepad 1 have standard forward controls over the robot!
     * @param defaultSpeedFactor The factor to multiple the speed by normally
     */
    public void updateDrive(double defaultSpeedFactor) {

        // If the OpMode didn't specifically initialize motors with settings, call the default one
        if(!hasUpdatedDrive && !hasInitializedMotors)
            initializeDriveMotors();
        hasUpdatedDrive = true;


        // Set drive state
        status = RobotStatus.DRIVING;

        // This is tuned to counteract imperfect strafing
        double strafingCo = 1.5;


        // Our input vars
        double x, y, ry, rx;

        x = -gamepad1.left_stick_x * strafingCo;
        y = gamepad1.left_stick_y;
        ry = -gamepad1.right_stick_y;
        rx = gamepad1.right_stick_x;

        // Power variables
        double fl = 0, fr = 0, bl = 0, br = 0;

        fl = y + x + rx;
        bl = y - x + rx;
        fr = y - x - rx;
        br = y + x - rx;

        // Making sure our speeds are in capped at -1, 1
        if (Math.abs(fl) > 1 || Math.abs(bl) > 1 ||
                Math.abs(fr) > 1 || Math.abs(fl) > 1 ) {
            // Find the largest power
            double max = 0;
            max = Math.max(Math.abs(fl), Math.abs(bl));
            max = Math.max(Math.abs(fr), max);
            max = Math.max(Math.abs(br), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            fl /= max;
            bl /= max;
            fr /= max;
            br /= max;
        }

        if(gamepad1.left_bumper && gamepad1.right_bumper) {
            fl *= defaultSpeedFactor*(1/5.0);
            fr *= defaultSpeedFactor*(1/5.0);
            bl *= defaultSpeedFactor*(1/5.0);
            br *= defaultSpeedFactor*(1/5.0);
        } else if(gamepad1.left_bumper || gamepad1.right_bumper) {
            fl *= defaultSpeedFactor*(1/2.0);
            fr *= defaultSpeedFactor*(1/2.0);
            bl *= defaultSpeedFactor*(1/2.0);
            br *= defaultSpeedFactor*(1/2.0);
        } else {
            fl *= defaultSpeedFactor;
            fr *= defaultSpeedFactor;
            bl *= defaultSpeedFactor;
            br *= defaultSpeedFactor;
        }

        motors[1].setPower(fr);
        motors[0].setPower(fl);
        motors[2].setPower(bl);
        motors[3].setPower(br);
    }

    /**
     * Our main driving method: uses one of a couple of drive algorithms to calculate and assign motor powers
     * @usage Only call this method after initialization and instantiating the robot
     * This algorithm makes gamepad 1 have standard forward controls over the robot!
     * @param precisionModeFactor The factor to multiple the speed by if ONE of the bumpers are held down
     * @param slowmodeFactor The factor to multiple the speed by if BOTH of the bumpers are held down
     * @param defaultSpeedFactor The factor to multiple the speed by normally
     */
    public void updateDrive(double defaultSpeedFactor, double precisionModeFactor, double slowmodeFactor) {

        // If the OpMode didn't specifically initialize motors with settings, call the default one
        if(!hasUpdatedDrive && !hasInitializedMotors)
            initializeDriveMotors();
        hasUpdatedDrive = true;


        // Set drive state
        status = RobotStatus.DRIVING;

        // This is tuned to counteract imperfect strafing
        double strafingCo = 1.5;


        // Our input vars
        double x, y, ry, rx;

        x = -gamepad1.left_stick_x * strafingCo;
        y = gamepad1.left_stick_y;
        ry = -gamepad1.right_stick_y;
        rx = gamepad1.right_stick_x;

        // Power variables
        double fl = 0, fr = 0, bl = 0, br = 0;

        fl = y + x + rx;
        bl = y - x + rx;
        fr = y - x - rx;
        br = y + x - rx;

        // Making sure our speeds are in capped at -1, 1
        if (Math.abs(fl) > 1 || Math.abs(bl) > 1 ||
                Math.abs(fr) > 1 || Math.abs(fl) > 1 ) {
            // Find the largest power
            double max = 0;
            max = Math.max(Math.abs(fl), Math.abs(bl));
            max = Math.max(Math.abs(fr), max);
            max = Math.max(Math.abs(br), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            fl /= max;
            bl /= max;
            fr /= max;
            br /= max;
        }

        if(gamepad1.left_bumper && gamepad1.right_bumper) {
            fl *= defaultSpeedFactor*(1/slowmodeFactor);
            fr *= defaultSpeedFactor*(1/slowmodeFactor);
            bl *= defaultSpeedFactor*(1/slowmodeFactor);
            br *= defaultSpeedFactor*(1/slowmodeFactor);
        } else if(gamepad1.left_bumper || gamepad1.right_bumper) {
            fl *= defaultSpeedFactor*(1/precisionModeFactor);
            fr *= defaultSpeedFactor*(1/precisionModeFactor);
            bl *= defaultSpeedFactor*(1/precisionModeFactor);
            br *= defaultSpeedFactor*(1/precisionModeFactor);
        } else {
            fl *= defaultSpeedFactor;
            fr *= defaultSpeedFactor;
            bl *= defaultSpeedFactor;
            br *= defaultSpeedFactor;
        }

        motors[1].setPower(fr);
        motors[0].setPower(fl);
        motors[2].setPower(bl);
        motors[3].setPower(br);
    }

    /**
     * Our main driving method: uses one of a couple of drive algorithms to calculate and assign motor powers
     * @usage Only call this method after initialization and instantiating the robot
     * @param useController1 Can controller1 drive?
     * @param useController2 Can controller2 drive?
     * @param controller1Dir Which direction should controller1 drive?
     * @param controller2Dir Which direction should controller2 drive?
     * @param controller2CanOverride Can controller2 override controller1 using the {@link RobotConfig#NUCLEAR_KEY}?
     * @param defaultSpeedFactor The factor to multiple the speed by normally
     */
    @Experimental
    public void updateDrive(OptimizedController controller1, OptimizedController controller2, boolean useController1, boolean useController2, double defaultSpeedFactor, RobotDirection controller1Dir, RobotDirection controller2Dir, boolean controller2CanOverride) {

        // If the OpMode didn't specifically initialize motors with settings, call the default one
        if(!hasUpdatedDrive && !hasInitializedMotors)
            initializeDriveMotors();
        hasUpdatedDrive = true;


        // Set drive state
        status = RobotStatus.DRIVING;

        // This is tuned to counteract imperfect strafing
        double strafingCo = 1.5;


        // Our input vars
        double x = 0, y = 0, ry = 0, rx = 0;

        boolean usingController1 = false;

        if(((!controller1.isBeingUsed() || !useController1) || (controller2.getBool(RobotConfig.NUCLEAR_KEY) && controller2CanOverride)) && useController2) {
            usingController1 = false;
            if(controller2Dir == RobotDirection.FRONT) {
                x = controller2.getFloat(OptimizedController.Key.LEFT_STICK_X) * strafingCo;
                y = -controller2.getFloat(OptimizedController.Key.LEFT_STICK_Y);
                ry = -controller2.getFloat(OptimizedController.Key.RIGHT_STICK_Y);
                rx = controller2.getFloat(OptimizedController.Key.RIGHT_STICK_X);
            } else if(controller2Dir == RobotDirection.BACK) {
                x = -controller2.getFloat(OptimizedController.Key.LEFT_STICK_X) * strafingCo;
                y = controller2.getFloat(OptimizedController.Key.LEFT_STICK_Y);
                ry = -controller2.getFloat(OptimizedController.Key.RIGHT_STICK_Y);
                rx = controller2.getFloat(OptimizedController.Key.RIGHT_STICK_X);
            } else if(controller2Dir == RobotDirection.LEFT) {
                x = controller2.getFloat(OptimizedController.Key.LEFT_STICK_Y) * strafingCo;
                y = controller2.getFloat(OptimizedController.Key.LEFT_STICK_X);
                ry = -controller2.getFloat(OptimizedController.Key.RIGHT_STICK_Y);
                rx = controller2.getFloat(OptimizedController.Key.RIGHT_STICK_X);
            } else {
                x = -controller2.getFloat(OptimizedController.Key.LEFT_STICK_Y) * strafingCo;
                y = -controller2.getFloat(OptimizedController.Key.LEFT_STICK_X);
                ry = -controller2.getFloat(OptimizedController.Key.RIGHT_STICK_Y);
                rx = controller2.getFloat(OptimizedController.Key.RIGHT_STICK_X);
            }
        } else if(useController1){
            usingController1 = true;
            if(controller1Dir == RobotDirection.FRONT) {
                x = controller1.getFloat(OptimizedController.Key.LEFT_STICK_X) * strafingCo;
                y = -controller1.getFloat(OptimizedController.Key.LEFT_STICK_Y);
                ry = -controller1.getFloat(OptimizedController.Key.RIGHT_STICK_Y);
                rx = controller1.getFloat(OptimizedController.Key.RIGHT_STICK_X);
            } else if(controller1Dir == RobotDirection.BACK){
                x = -controller1.getFloat(OptimizedController.Key.LEFT_STICK_X) * strafingCo;
                y = controller1.getFloat(OptimizedController.Key.LEFT_STICK_Y);
                ry = -controller1.getFloat(OptimizedController.Key.RIGHT_STICK_Y);
                rx = controller1.getFloat(OptimizedController.Key.RIGHT_STICK_X);
            } else if(controller1Dir == RobotDirection.LEFT){
                x = controller1.getFloat(OptimizedController.Key.LEFT_STICK_Y) * strafingCo;
                y = controller1.getFloat(OptimizedController.Key.LEFT_STICK_X);
                ry = -controller1.getFloat(OptimizedController.Key.RIGHT_STICK_Y);
                rx = controller1.getFloat(OptimizedController.Key.RIGHT_STICK_X);
            } else {
                x = -controller1.getFloat(OptimizedController.Key.LEFT_STICK_Y) * strafingCo;
                y = -controller1.getFloat(OptimizedController.Key.LEFT_STICK_X);
                ry = -controller1.getFloat(OptimizedController.Key.RIGHT_STICK_Y);
                rx = controller1.getFloat(OptimizedController.Key.RIGHT_STICK_X);
            }
        }

        // Power variables
        double fl = 0, fr = 0, bl = 0, br = 0;

        fl = y + x + rx;
        bl = y - x + rx;
        fr = y - x - rx;
        br = y + x - rx;

        // Making sure our speeds are in capped at -1, 1
        if (Math.abs(fl) > 1 || Math.abs(bl) > 1 ||
                Math.abs(fr) > 1 || Math.abs(fl) > 1 ) {
            // Find the largest power
            double max = 0;
            max = Math.max(Math.abs(fl), Math.abs(bl));
            max = Math.max(Math.abs(fr), max);
            max = Math.max(Math.abs(br), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            fl /= max;
            bl /= max;
            fr /= max;
            br /= max;
        }

        if(usingController1) {
            if(controller1.getBool(OptimizedController.Key.LEFT_BUMPER) && controller1.getBool(OptimizedController.Key.RIGHT_BUMPER)) {
                fl *= defaultSpeedFactor*(1/5.0);
                fr *= defaultSpeedFactor*(1/5.0);
                bl *= defaultSpeedFactor*(1/5.0);
                br *= defaultSpeedFactor*(1/5.0);
            } else if(controller1.getBool(OptimizedController.Key.LEFT_BUMPER) || controller1.getBool(OptimizedController.Key.RIGHT_BUMPER)) {
                fl *= defaultSpeedFactor*(1/2.0);
                fr *= defaultSpeedFactor*(1/2.0);
                bl *= defaultSpeedFactor*(1/2.0);
                br *= defaultSpeedFactor*(1/2.0);
            } else {
                fl *= defaultSpeedFactor;
                fr *= defaultSpeedFactor;
                bl *= defaultSpeedFactor;
                br *= defaultSpeedFactor;
            }
        } else {
            if(controller2.getBool(OptimizedController.Key.LEFT_BUMPER) && controller2.getBool(OptimizedController.Key.RIGHT_BUMPER)) {
                fl *= defaultSpeedFactor*(1/5.0);
                fr *= defaultSpeedFactor*(1/5.0);
                bl *= defaultSpeedFactor*(1/5.0);
                br *= defaultSpeedFactor*(1/5.0);
            } else if(controller2.getBool(OptimizedController.Key.LEFT_BUMPER) || controller2.getBool(OptimizedController.Key.RIGHT_BUMPER)) {
                fl *= defaultSpeedFactor*(1/2.0);
                fr *= defaultSpeedFactor*(1/2.0);
                bl *= defaultSpeedFactor*(1/2.0);
                br *= defaultSpeedFactor*(1/2.0);
            } else {
                fl *= defaultSpeedFactor;
                fr *= defaultSpeedFactor;
                bl *= defaultSpeedFactor;
                br *= defaultSpeedFactor;
            }
        }

        motors[1].setPower(fr);
        motors[0].setPower(fl);
        motors[2].setPower(bl);
        motors[3].setPower(br);
    }

    /**
     * Our main driving method: uses one of a couple of drive algorithms to calculate and assign motor powers
     * @usage Only call this method after initialization and instantiating the robot
     * @param useController1 Can controller1 drive?
     * @param useController2 Can controller2 drive?
     * @param controller1Dir Which direction should controller1 drive?
     * @param controller2Dir Which direction should controller2 drive?
     * @param controller2CanOverride Can controller2 override controller1 using the {@link RobotConfig#NUCLEAR_KEY}?
     * @param defaultSpeedFactor The factor to multiple the speed by normally
     * @param precisionModeFactor The factor to multiple the speed by if ONE of the bumpers are held down
     * @param slowmodeFactor The factor to multiple the speed by if BOTH of the bumpers are held down
     */
    @Experimental
    public void updateDrive(OptimizedController controller1, OptimizedController controller2, boolean useController1, boolean useController2, double defaultSpeedFactor, double precisionModeFactor, double slowmodeFactor, RobotDirection controller1Dir, RobotDirection controller2Dir, boolean controller2CanOverride) {

        // If the OpMode didn't specifically initialize motors with settings, call the default one
        if(!hasUpdatedDrive && !hasInitializedMotors)
            initializeDriveMotors();
        hasUpdatedDrive = true;


        // Set drive state
        status = RobotStatus.DRIVING;

        // This is tuned to counteract imperfect strafing
        double strafingCo = 1.5;


        // Our input vars
        double x = 0, y = 0, ry = 0, rx = 0;

        boolean usingController1 = false;

        if(((!controller1.isBeingUsed() || !useController1) || (controller2.getBool(RobotConfig.NUCLEAR_KEY) && controller2CanOverride)) && useController2) {
            usingController1 = false;
            if(controller2Dir == RobotDirection.FRONT) {
                x = controller2.getFloat(OptimizedController.Key.LEFT_STICK_X) * strafingCo;
                y = -controller2.getFloat(OptimizedController.Key.LEFT_STICK_Y);
                ry = -controller2.getFloat(OptimizedController.Key.RIGHT_STICK_Y);
                rx = controller2.getFloat(OptimizedController.Key.RIGHT_STICK_X);
            } else if(controller2Dir == RobotDirection.BACK) {
                x = -controller2.getFloat(OptimizedController.Key.LEFT_STICK_X) * strafingCo;
                y = controller2.getFloat(OptimizedController.Key.LEFT_STICK_Y);
                ry = -controller2.getFloat(OptimizedController.Key.RIGHT_STICK_Y);
                rx = controller2.getFloat(OptimizedController.Key.RIGHT_STICK_X);
            } else if(controller2Dir == RobotDirection.LEFT) {
                x = controller2.getFloat(OptimizedController.Key.LEFT_STICK_Y) * strafingCo;
                y = controller2.getFloat(OptimizedController.Key.LEFT_STICK_X);
                ry = -controller2.getFloat(OptimizedController.Key.RIGHT_STICK_Y);
                rx = controller2.getFloat(OptimizedController.Key.RIGHT_STICK_X);
            } else {
                x = -controller2.getFloat(OptimizedController.Key.LEFT_STICK_Y) * strafingCo;
                y = -controller2.getFloat(OptimizedController.Key.LEFT_STICK_X);
                ry = -controller2.getFloat(OptimizedController.Key.RIGHT_STICK_Y);
                rx = controller2.getFloat(OptimizedController.Key.RIGHT_STICK_X);
            }
        } else if(useController1){
            usingController1 = true;
            if(controller1Dir == RobotDirection.FRONT) {
                x = controller1.getFloat(OptimizedController.Key.LEFT_STICK_X) * strafingCo;
                y = -controller1.getFloat(OptimizedController.Key.LEFT_STICK_Y);
                ry = -controller1.getFloat(OptimizedController.Key.RIGHT_STICK_Y);
                rx = controller1.getFloat(OptimizedController.Key.RIGHT_STICK_X);
            } else if(controller1Dir == RobotDirection.BACK){
                x = -controller1.getFloat(OptimizedController.Key.LEFT_STICK_X) * strafingCo;
                y = controller1.getFloat(OptimizedController.Key.LEFT_STICK_Y);
                ry = -controller1.getFloat(OptimizedController.Key.RIGHT_STICK_Y);
                rx = controller1.getFloat(OptimizedController.Key.RIGHT_STICK_X);
            } else if(controller1Dir == RobotDirection.LEFT){
                x = controller1.getFloat(OptimizedController.Key.LEFT_STICK_Y) * strafingCo;
                y = controller1.getFloat(OptimizedController.Key.LEFT_STICK_X);
                ry = -controller1.getFloat(OptimizedController.Key.RIGHT_STICK_Y);
                rx = controller1.getFloat(OptimizedController.Key.RIGHT_STICK_X);
            } else {
                x = -controller1.getFloat(OptimizedController.Key.LEFT_STICK_Y) * strafingCo;
                y = -controller1.getFloat(OptimizedController.Key.LEFT_STICK_X);
                ry = -controller1.getFloat(OptimizedController.Key.RIGHT_STICK_Y);
                rx = controller1.getFloat(OptimizedController.Key.RIGHT_STICK_X);
            }
        }

        // Power variables
        double fl = 0, fr = 0, bl = 0, br = 0;

        fl = y + x + rx;
        bl = y - x + rx;
        fr = y - x - rx;
        br = y + x - rx;

        // Making sure our speeds are in capped at -1, 1
        if (Math.abs(fl) > 1 || Math.abs(bl) > 1 ||
                Math.abs(fr) > 1 || Math.abs(fl) > 1 ) {
            // Find the largest power
            double max = 0;
            max = Math.max(Math.abs(fl), Math.abs(bl));
            max = Math.max(Math.abs(fr), max);
            max = Math.max(Math.abs(br), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            fl /= max;
            bl /= max;
            fr /= max;
            br /= max;
        }

        if(usingController1) {
            if(controller1.getBool(OptimizedController.Key.LEFT_BUMPER) && controller1.getBool(OptimizedController.Key.RIGHT_BUMPER)) {
                fl *= defaultSpeedFactor*(1/slowmodeFactor);
                fr *= defaultSpeedFactor*(1/slowmodeFactor);
                bl *= defaultSpeedFactor*(1/slowmodeFactor);
                br *= defaultSpeedFactor*(1/slowmodeFactor);
            } else if(controller1.getBool(OptimizedController.Key.LEFT_BUMPER) || controller1.getBool(OptimizedController.Key.RIGHT_BUMPER)) {
                fl *= defaultSpeedFactor*(1/precisionModeFactor);
                fr *= defaultSpeedFactor*(1/precisionModeFactor);
                bl *= defaultSpeedFactor*(1/precisionModeFactor);
                br *= defaultSpeedFactor*(1/precisionModeFactor);
            } else {
                fl *= defaultSpeedFactor;
                fr *= defaultSpeedFactor;
                bl *= defaultSpeedFactor;
                br *= defaultSpeedFactor;
            }
        } else {
            if(controller2.getBool(OptimizedController.Key.LEFT_BUMPER) && controller2.getBool(OptimizedController.Key.RIGHT_BUMPER)) {
                fl *= defaultSpeedFactor*(1/slowmodeFactor);
                fr *= defaultSpeedFactor*(1/slowmodeFactor);
                bl *= defaultSpeedFactor*(1/slowmodeFactor);
                br *= defaultSpeedFactor*(1/slowmodeFactor);
            } else if(controller2.getBool(OptimizedController.Key.LEFT_BUMPER) || controller2.getBool(OptimizedController.Key.RIGHT_BUMPER)) {
                fl *= defaultSpeedFactor*(1/precisionModeFactor);
                fr *= defaultSpeedFactor*(1/precisionModeFactor);
                bl *= defaultSpeedFactor*(1/precisionModeFactor);
                br *= defaultSpeedFactor*(1/precisionModeFactor);
            } else {
                fl *= defaultSpeedFactor;
                fr *= defaultSpeedFactor;
                bl *= defaultSpeedFactor;
                br *= defaultSpeedFactor;
            }
        }

        motors[1].setPower(fr);
        motors[0].setPower(fl);
        motors[2].setPower(bl);
        motors[3].setPower(br);
    }

    /**
     * Grab an encoder
     * @param name Name of encoder
     */
    public Encoder getEncoder(String name) {
        return new Encoder(internalMap.get(DcMotorEx.class, name));
    }

    /**
     * Below is a series of methods to change the settings of the motors. Ignore mostly, these are all used in another method shown here:
     * @link #initializeDriveMotors()
     * @param withEncoder
     */
    protected void runWithEncoder(boolean withEncoder){
        if(withEncoder) {
            for(int i = 0; i < motors.length; i++) {
                if(!disabledMotors[i]) {
                    motors[i].setMode(RunMode.STOP_AND_RESET_ENCODER);
                    motors[i].setMode(RunMode.RUN_USING_ENCODER);
                }
            }
        }else {
            for(int i = 0; i < motors.length; i++) {
                if(!disabledMotors[i]) {
                    motors[i].setMode(RunMode.RUN_WITHOUT_ENCODER);
                }
            }
        }
    }

    private void setMotorBrakeType(ZeroPowerBehavior type) {
        for(int i = 0; i < motors.length; i++) {
            if(!disabledMotors[i]) {
                motors[i].setZeroPowerBehavior(type);
            }
        }
    }

    protected void motorDir(boolean forward){
        if(forward){
            for(int i = 0; i < motors.length; i++) {
                if(!disabledMotors[i]) {
                    motors[i].setDirection(RobotConfig.motorDirections[i]);
                }
            }
        } else {
            for(int i = 0; i < motors.length; i++) {
                if(!disabledMotors[i]) {
                    if(RobotConfig.motorDirections[i] == Direction.FORWARD)
                        motors[i].setDirection(Direction.REVERSE);
                    else
                        motors[i].setDirection(Direction.FORWARD);
                }
            }
        }
    }

    /**
     * This method is used for testing purposes (if you want to swap another motor into the same stop as a drive motor, disable it so no error occurs)
     * @param FLDisabled Whether or not the FLMotor is disabled
     * @param FRDisabled Whether or not the FRMotor is disabled
     * @param BLDisabled Whether or not the BLMotor is disabled
     * @param BRDisabled Whether or not the BRMotor is disabled
     */
    public void disableDriveMotors(boolean FLDisabled, boolean FRDisabled, boolean BLDisabled, boolean BRDisabled) {
        boolean[] a = {FLDisabled, FRDisabled, BLDisabled, BRDisabled};
        disabledMotors = a;
    }

    /**
     * Returns the status of the robot. Currently has not much use but could be helpful in the future
     * @return
     */
    public RobotStatus getStatus(){
        return status;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param name The name of the motor in the hardwareMap
     * @return The DCMotor
     */
    public DcMotor getMotor(String name) {
        DcMotor motor = internalMap.dcMotor.get(name);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param name The name of the motor in the hardwareMap
     * @return The DCMotor
     */
    public DcMotorEx getMotorEx(String name) {
        DcMotorEx motor = internalMap.get(DcMotorEx.class, name);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param name The name of the motor in the hardwareMap
     * @param runmode The runmode to use for this motor
     * @return The DCMotor
     */
    public DcMotor getMotor(String name, RunMode runmode) {
        DcMotor motor = internalMap.dcMotor.get(name);
        motor.setMode(runmode);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param name The name of the motor in the hardwareMap
     * @return The DCMotor
     */
    public DcMotorEx getMotorEx(String name, RunMode runmode) {
        DcMotorEx motor = internalMap.get(DcMotorEx.class, name);
        motor.setMode(runmode);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param name The name of the motor in the hardwareMap
     * @param runmode The runmode to use for this motor
     * @param direction The direction for this motor
     * @return The DCMotor
     */
    public DcMotor getMotor(String name, RunMode runmode, Direction direction) {
        DcMotor motor = internalMap.dcMotor.get(name);
        motor.setMode(runmode);
        motor.setDirection(direction);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param name The name of the motor in the hardwareMap
     * @param runmode The runmode to use for this motor
     * @param direction The direction for this motor
     * @return The DCMotor
     */
    public DcMotorEx getMotorEx(String name, RunMode runmode, Direction direction) {
        DcMotorEx motor = internalMap.get(DcMotorEx.class, name);
        motor.setMode(runmode);
        motor.setDirection(direction);
        return motor;
    }

    /**
     * Turns Rev BlinkinLed Lights into the confetti light pattern
     */
    @Experimental
    @OldCode
    public void runLights(RevBlinkinLedDriver.BlinkinPattern pattern){
        RevBlinkinLedDriver ledLights = internalMap.get(RevBlinkinLedDriver.class, "light"/*check defined name*/);
        ledLights.setPattern(pattern);
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param name The name of the motor in the hardwareMap
     * @param runmode The runmode to use for this motor
     * @param direction The direction for this motor
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotor getMotor(String name, RunMode runmode, Direction direction, ZeroPowerBehavior brakeMode) {
        DcMotor motor = internalMap.dcMotor.get(name);
        motor.setMode(runmode);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(brakeMode);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param name The name of the motor in the hardwareMap
     * @param runmode The runmode to use for this motor
     * @param direction The direction for this motor
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotorEx getMotorEx(String name, RunMode runmode, Direction direction, ZeroPowerBehavior brakeMode) {
        DcMotorEx motor = internalMap.get(DcMotorEx.class, name);
        motor.setMode(runmode);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(brakeMode);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param name The name of the motor in the hardwareMap
     * @param runmode The runmode to use for this motor
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotor getMotor(String name, RunMode runmode, ZeroPowerBehavior brakeMode) {
        DcMotor motor = internalMap.dcMotor.get(name);
        motor.setMode(runmode);
        motor.setZeroPowerBehavior(brakeMode);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param name The name of the motor in the hardwareMap
     * @param runmode The runmode to use for this motor
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotorEx getMotorEx(String name, RunMode runmode, ZeroPowerBehavior brakeMode) {
        DcMotorEx motor = internalMap.get(DcMotorEx.class, name);
        motor.setMode(runmode);
        motor.setZeroPowerBehavior(brakeMode);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param name The name of the motor in the hardwareMap
     * @param direction The direction for this motor
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotor getMotor(String name, Direction direction, ZeroPowerBehavior brakeMode) {
        DcMotor motor = internalMap.dcMotor.get(name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(brakeMode);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param name The name of the motor in the hardwareMap
     * @param direction The direction for this motor
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotorEx getMotorEx(String name, Direction direction, ZeroPowerBehavior brakeMode) {
        DcMotorEx motor = internalMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(brakeMode);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param name The name of the motor in the hardwareMap
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotor getMotor(String name, ZeroPowerBehavior brakeMode) {
        DcMotor motor = internalMap.dcMotor.get(name);
        motor.setZeroPowerBehavior(brakeMode);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param name The name of the motor in the hardwareMap
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotorEx getMotorEx(String name, ZeroPowerBehavior brakeMode) {
        DcMotorEx motor = internalMap.get(DcMotorEx.class, name);
        motor.setZeroPowerBehavior(brakeMode);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param name The name of the motor in the hardwareMap
     * @param direction The direction for this motor
     * @return The DCMotor
     */
    public DcMotor getMotor(String name, Direction direction) {
        DcMotor motor = internalMap.dcMotor.get(name);
        motor.setDirection(direction);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param name The name of the motor in the hardwareMap
     * @param direction The direction for this motor
     * @return The DCMotor
     */
    public DcMotorEx getMotorEx(String name, Direction direction) {
        DcMotorEx motor = internalMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        return motor;
    }


    /**
     * Searches through our map of aliases and looks for which hardware map name it falls under
     * @param alias The alias of the hardware component
     * @return The true hardware name
     */
    private String findMapNameByAlias(String alias) {
        for(String key : aliasMap.keySet()) {
            if(alias.equals(key))
                return key;
            for(String val : aliasMap.get(key)) {
                if(val.equals(alias))
                    return key;
            }
        }
        return null;
    }


    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param alias The alias of the motor in the hardwareMap
     * @return The DCMotor
     */
    public DcMotor getMotorByAlias(String alias) {
        DcMotor motor = internalMap.dcMotor.get(findMapNameByAlias(alias));
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param alias The alias of the motor in the hardwareMap
     * @param runmode The runmode to use for this motor
     * @return The DCMotor
     */
    public DcMotor getMotorByAlias(String alias, RunMode runmode) {
        DcMotor motor = internalMap.dcMotor.get(findMapNameByAlias(alias));
        motor.setMode(runmode);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param alias The alias of the motor in the hardwareMap
     * @param runmode The runmode to use for this motor
     * @param direction The direction for this motor
     * @return The DCMotor
     */
    public DcMotor getMotorByAlias(String alias, RunMode runmode, Direction direction) {
        DcMotor motor = internalMap.dcMotor.get(findMapNameByAlias(alias));
        motor.setMode(runmode);
        motor.setDirection(direction);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param alias The alias of the motor in the hardwareMap
     * @param runmode The runmode to use for this motor
     * @param direction The direction for this motor
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotor getMotorByAlias(String alias, RunMode runmode, Direction direction, ZeroPowerBehavior brakeMode) {
        DcMotor motor = internalMap.dcMotor.get(findMapNameByAlias(alias));
        motor.setMode(runmode);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(brakeMode);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param alias The alias of the motor in the hardwareMap
     * @param runmode The runmode to use for this motor
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotor getMotorByAlias(String alias, RunMode runmode, ZeroPowerBehavior brakeMode) {
        DcMotor motor = internalMap.dcMotor.get(findMapNameByAlias(alias));
        motor.setMode(runmode);
        motor.setZeroPowerBehavior(brakeMode);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param alias The alias of the motor in the hardwareMap
     * @param direction The direction for this motor
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotor getMotorByAlias(String alias, Direction direction, ZeroPowerBehavior brakeMode) {
        DcMotor motor = internalMap.dcMotor.get(findMapNameByAlias(alias));
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(brakeMode);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param alias The alias of the motor in the hardwareMap
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotor getMotorByAlias(String alias, ZeroPowerBehavior brakeMode) {
        DcMotor motor = internalMap.dcMotor.get(findMapNameByAlias(alias));
        motor.setZeroPowerBehavior(brakeMode);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param alias The alias of the motor in the hardwareMap
     * @param direction The direction for this motor
     * @return The DCMotor
     */
    public DcMotor getMotorByAlias(String alias, Direction direction) {
        DcMotor motor = internalMap.dcMotor.get(findMapNameByAlias(alias));
        motor.setDirection(direction);
        return motor;
    }












    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param alias The alias of the motor in the hardwareMap
     * @return The DCMotor
     */
    public DcMotorEx getMotorExByAlias(String alias) {
        DcMotorEx motor = internalMap.get(DcMotorEx.class, findMapNameByAlias(alias));
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param alias The alias of the motor in the hardwareMap
     * @param runmode The runmode to use for this motor
     * @return The DCMotor
     */
    public DcMotorEx getMotorExByAlias(String alias, RunMode runmode) {
        DcMotorEx motor = internalMap.get(DcMotorEx.class, findMapNameByAlias(alias));
        motor.setMode(runmode);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param alias The alias of the motor in the hardwareMap
     * @param runmode The runmode to use for this motor
     * @param direction The direction for this motor
     * @return The DCMotor
     */
    public DcMotorEx getMotorExByAlias(String alias, RunMode runmode, Direction direction) {
        DcMotorEx motor = internalMap.get(DcMotorEx.class, findMapNameByAlias(alias));
        motor.setMode(runmode);
        motor.setDirection(direction);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param alias The alias of the motor in the hardwareMap
     * @param runmode The runmode to use for this motor
     * @param direction The direction for this motor
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotorEx getMotorExByAlias(String alias, RunMode runmode, Direction direction, ZeroPowerBehavior brakeMode) {
        DcMotorEx motor = internalMap.get(DcMotorEx.class, findMapNameByAlias(alias));
        motor.setMode(runmode);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(brakeMode);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param alias The alias of the motor in the hardwareMap
     * @param runmode The runmode to use for this motor
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotorEx getMotorExByAlias(String alias, RunMode runmode, ZeroPowerBehavior brakeMode) {
        DcMotorEx motor = internalMap.get(DcMotorEx.class, findMapNameByAlias(alias));
        motor.setMode(runmode);
        motor.setZeroPowerBehavior(brakeMode);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param alias The alias of the motor in the hardwareMap
     * @param direction The direction for this motor
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotorEx getMotorExByAlias(String alias, Direction direction, ZeroPowerBehavior brakeMode) {
        DcMotorEx motor = internalMap.get(DcMotorEx.class, findMapNameByAlias(alias));
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(brakeMode);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param alias The alias of the motor in the hardwareMap
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotorEx getMotorExByAlias(String alias, ZeroPowerBehavior brakeMode) {
        DcMotorEx motor = internalMap.get(DcMotorEx.class, findMapNameByAlias(alias));
        motor.setZeroPowerBehavior(brakeMode);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     * @param alias The alias of the motor in the hardwareMap
     * @param direction The direction for this motor
     * @return The DCMotor
     */
    public DcMotorEx getMotorExByAlias(String alias, Direction direction) {
        DcMotorEx motor = internalMap.get(DcMotorEx.class, findMapNameByAlias(alias));
        motor.setDirection(direction);
        return motor;
    }

    /**
     * Returns a servo to be used in an OpMode
     * @param name The hardwareMap name of the servo
     * @return The servo
     */
    public Servo getServo(String name) {
        Servo servo = internalMap.servo.get(name);
        return servo;
    }

    /**
     * Returns a servo to be used in an OpMode
     * @param name The hardwareMap name of the servo
     * @param initialPosition The initialize position for this servo to assume
     * @return The servo
     */
    public Servo getServo(String name, double initialPosition) {
        Servo servo = internalMap.servo.get(name);
        servo.setPosition(initialPosition);
        return servo;
    }

    /**
     * Returns an ODS sensor to be used in an OpMode
     * @param name The hardwareMap name of the ODS sensor
     * @return The ODS Sensor
     */
    public OpticalDistanceSensor getDistanceSensor(String name) {
        return internalMap.opticalDistanceSensor.get(name);
    }

    /**
     * Returns an instance of the DriveFunctions Class, which provides a series of primitive functions for autonomous
     * @return Obj of the DriveFunctions class
     */
    public OptimizedDriveFunctions getDriveFunctions(){
        return functions;
    }

    /**
     * Returns the drive mode currently being used by the robot. Not much of a use for it, but kept just in-case.
     * To see a list of available drive modes, go here {@link DriveMode}
     * @return An ENUM representing the drive mode
     */
    public DriveMode getDriveMode(){
        return driveMode;
    }

    /**
     * Call this method to change the current drive mode of the robot
     * @usage You really should only be calling this method during initialization unless you are doing something special.
     * @param mode An ENUM of type {@link DriveMode}
     */
    public void setDriveMode(DriveMode mode){
        driveMode = mode;
    }

    /**
     * This method is very unique: If called explicitly before calling the first instance of updateDrive(), then it can be customized to the programmer's liking.
     * If not, the parameter-less version will run with default settings for teleop
     */
    protected void initializeDriveMotors(){
        if(!disabledMotors[0]) {
            motors[0] = internalMap.dcMotor.get("frontLeftMotor");
        }
        if(!disabledMotors[1]) {
            motors[1] = internalMap.dcMotor.get("frontRightMotor");
        }
        if(!disabledMotors[2]) {
            motors[2] = internalMap.dcMotor.get("backLeftMotor");
        }
        if(!disabledMotors[3]) {
            motors[3] = internalMap.dcMotor.get("backRightMotor");
        }

        runWithEncoder(false);

        motorDir(true);

        setMotorBrakeType(ZeroPowerBehavior.FLOAT);
    }

    public void initializeDriveMotors(boolean isForward, boolean runsEncoders, ZeroPowerBehavior brakeType){

        hasInitializedMotors = true;

        if(!disabledMotors[0]) {
            motors[0] = internalMap.dcMotor.get("frontLeftMotor");
        }
        if(!disabledMotors[1]) {
            motors[1] = internalMap.dcMotor.get("frontRightMotor");
        }
        if(!disabledMotors[2]) {
            motors[2] = internalMap.dcMotor.get("backLeftMotor");
        }
        if(!disabledMotors[3]) {
            motors[3] = internalMap.dcMotor.get("backRightMotor");
        }

        runWithEncoder(runsEncoders);

        motorDir(isForward);

        setMotorBrakeType(brakeType);
    }

    public void initializeDriveMotors(boolean isForward, boolean runsEncoders){

        hasInitializedMotors = true;

        if(!disabledMotors[0]) {
            motors[0] = internalMap.dcMotor.get("frontLeftMotor");
        }
        if(!disabledMotors[1]) {
            motors[1] = internalMap.dcMotor.get("frontRightMotor");
        }
        if(!disabledMotors[2]) {
            motors[2] = internalMap.dcMotor.get("backLeftMotor");
        }
        if(!disabledMotors[3]) {
            motors[3] = internalMap.dcMotor.get("backRightMotor");
        }

        runWithEncoder(runsEncoders);

        motorDir(isForward);

        setMotorBrakeType(ZeroPowerBehavior.FLOAT);
    }

    public void initializeDriveMotors(boolean isForward, ZeroPowerBehavior brakeType){

        hasInitializedMotors = true;

        if(!disabledMotors[0]) {
            motors[0] = internalMap.dcMotor.get("frontLeftMotor");
        }
        if(!disabledMotors[1]) {
            motors[1] = internalMap.dcMotor.get("frontRightMotor");
        }
        if(!disabledMotors[2]) {
            motors[2] = internalMap.dcMotor.get("backLeftMotor");
        }
        if(!disabledMotors[3]) {
            motors[3] = internalMap.dcMotor.get("backRightMotor");
        }

        runWithEncoder(false);

        motorDir(isForward);

        setMotorBrakeType(brakeType);
    }

    public void initializeDriveMotors(boolean runsEncoders){

        hasInitializedMotors = true;

        if(!disabledMotors[0]) {
            motors[0] = internalMap.dcMotor.get("frontLeftMotor");
        }
        if(!disabledMotors[1]) {
            motors[1] = internalMap.dcMotor.get("frontRightMotor");
        }
        if(!disabledMotors[2]) {
            motors[2] = internalMap.dcMotor.get("backLeftMotor");
        }
        if(!disabledMotors[3]) {
            motors[3] = internalMap.dcMotor.get("backRightMotor");
        }

        runWithEncoder(runsEncoders);

        motorDir(true);

        setMotorBrakeType(ZeroPowerBehavior.FLOAT);
    }
}
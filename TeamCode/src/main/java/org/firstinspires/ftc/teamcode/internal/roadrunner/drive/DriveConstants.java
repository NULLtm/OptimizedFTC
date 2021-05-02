package org.firstinspires.ftc.teamcode.internal.roadrunner.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
 * Constants shared between multiple drive types.
 *
 * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
 * fields may also be edited through the dashboard (connect to the robot's WiFi network and
 * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
 * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
 *
 * These are not the only parameters; some are located in the localizer classes, drive base classes,
 * and op modes themselves.
 */
@Config
public class DriveConstants {

    // These values are for your drive train motors/wheels NOT your odometry pods
    // TODO: Set the first four appropriately, and get a good estimate for track width
    // TODO: You will manually tune the track-width later
    public static final double TICKS_PER_REV = 537.6;
    public static final double MAX_RPM = 312;
    public static double WHEEL_RADIUS = 1.89; // inches.
    public static double GEAR_RATIO = 19.2; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 17.5; // inches. DOESN'T necessarily need to reflect the ACTUAL track width



    // IMU Name
    public static final String IMU_NAME = "imu";


    // For encoders on odometry ONLY
    // TODO: Set the first three appropriately, and get a good estimate of the track width
    // TODO: Track width will be tuned later
    public static final double ODOMETRY_TICKS_PER_REV = 8192; // ticks
    public static final double ODOMETRY_WHEEL_RADIUS = 0.75; // in
    // For standard dead wheels, this will be 1:1
    public static final double ODOMETRY_GEAR_RATIO = 1;
    public static final double ODOMETRY_FORWARD_OFFSET = 6.5; // in
    public static double ODOMETRY_TRACK_WIDTH = 14.655786; // in

    // TODO: Tune this for dead-wheel calibration
    public static double ODO_X_MULTIPLIER = 1.0202262; // Multiplier in the X direction (for left and right)
    public static double ODO_Y_MULTIPLIER = 1.02396659; // Multiplier in the Y direction (for strafe/lateral dir)


    // TODO: Set these to your hardware map motor names
    public static final String FRONT_RIGHT_MOTOR = "frontRightMotor";
    public static final String FRONT_LEFT_MOTOR = "frontLeftMotor";
    public static final String BACK_RIGHT_MOTOR = "backRightMotor";
    public static final String BACK_LEFT_MOTOR = "backLeftMotor";

    public static final String LEFT_ENCODER = "leftEncoder";
    public static final String RIGHT_ENCODER = "rightEncoder";
    public static final String STRAFE_ENCODER = "strafeEncoder";


    // TODO: Change motor directions {FL, FR, BL, BR}
    public static final DcMotorSimple.Direction[] motorDirections = {DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD};

    // TODO: If your hub is mounted vertically
    public static boolean HUB_MOUNTED_VERTICAL = false;



    // TODO: If you want to use the VeloPID controller, set the var below to true
    // Most people say this tuning method is TRASH
    // false = you will use feedforward tuning
    // true = you will use VelocityPID tuning
    public static final boolean RUN_USING_ENCODER = false;

    // TODO: If using VelPID controller, update this with your tuned constants
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));




    // BELOW is FeedForward Calibration only!!!

    // These values are for feedforward calibration
    // TODO: If you do not choose to using velPID controller, then you will tune these
    // HOW TO TUNE
    // -----------
    // 0.5. Run the ManualFeedForwardTuner OpMode
    // 1. Set all three values to 0
    // 2. Increase kV until the robot velocity graph (measured vs. target) shows that you are hitting the correct max velocity
    //
    //
    // 3. If you graph looks like triangles, increase the distance you travel VIA the dashboard (or the code itself)
    // 4. After making sure your measured velocity plateaus correctly, increase vStatic until the start delay is removed
    // 5. After that, increase kA until phase lag is removed (delay in the start of the trajectory!)

    // Other Helpful Tips
    // VALUE     Good starting Value    Example Working Value
    // kV             0.01                     0.017
    // kStatic        0.01                     0.05
    // kA             0.001          z          0.003
    public static double kV = 0.0178; // 0.016 also seems to work sometimes
    public static double kA = 0.006;
    public static double kStatic = 0.03;


    // 0.0176 -- 12.7 V
    // 0.0157 -- 13.2 V
    // 0.0152 -- 13.7??? CHECK AGAIN




    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. All distance units are
     * inches.
     */
    public static double MAX_VEL = 50;
    public static double MAX_ACCEL = 30;
    public static double MAX_ANG_VEL = 12.416;
    public static double MAX_ANG_ACCEL = Math.toRadians(60);


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}

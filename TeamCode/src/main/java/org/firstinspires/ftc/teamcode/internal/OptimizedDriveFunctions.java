package org.firstinspires.ftc.teamcode.internal;

import android.os.Build;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.jetbrains.annotations.NotNull;

import java.time.Duration;
import java.util.List;

import androidx.annotation.RequiresApi;

/**
 * This class adds many reusable autonomous and teleop opmode functions for you to use by the call of a method!
 *
 * @author Owen Boseley - Class of 2021
 */
public class OptimizedDriveFunctions {

    /**
     * Defining strafing directions
     */
    public enum Direction {
        LEFT, RIGHT
    }

    /**
     * Internal robot to use
     */
    private OptimizedRobot robot = null;

    /**
     * Default Constructor
     * @param robot The optimized robot to use
     */
    protected OptimizedDriveFunctions(OptimizedRobot robot){
            this.robot = robot;
    }

    /**
     * Drive linearly (forward/backward) at specified power
     * @param power Power to drive at
     */
    public void linearDrive(float power){
        robot.motors[0].setPower(power);
        robot.motors[1].setPower(power);
        robot.motors[2].setPower(power);
        robot.motors[3].setPower(power);
    }

    /**
     * Tank turning
     * @param direction Direction to turn
     * @param power Power to turn at
     */
    public void turn (Direction direction, double power){
        if(direction == Direction.LEFT){
            robot.motors[0].setPower(-power);
            robot.motors[1].setPower(power);
            robot.motors[2].setPower(-power);
            robot.motors[3].setPower(power);
        } else if (direction == Direction.RIGHT){
            robot.motors[0].setPower(power);
            robot.motors[1].setPower(-power);
            robot.motors[2].setPower(power);
            robot.motors[3].setPower(-power);
        } else {
            robot.motors[0].setPower(0);
            robot.motors[1].setPower(0);
            robot.motors[2].setPower(0);
            robot.motors[3].setPower(0);
        }
    }

    /**
     * Strafe in certain direction
     * @param direction direction to travel
     * @param power Power to strafe at (-1 to 1)
     */
    public void strafeLinear(Direction direction, double power){
        if(direction == Direction.LEFT){
            robot.motors[0].setPower(power);
            robot.motors[1].setPower(-power);
            robot.motors[2].setPower(-power);
            robot.motors[3].setPower(power);
        } else if(direction == Direction.RIGHT){
            robot.motors[0].setPower(-power);
            robot.motors[1].setPower(power);
            robot.motors[2].setPower(power);
            robot.motors[3].setPower(-power);
        } else {
            robot.motors[0].setPower(0);
            robot.motors[1].setPower(0);
            robot.motors[2].setPower(0);
            robot.motors[3].setPower(0);
        }
    }

    /**
     * Simple two way motor controls -- generally this form of input is used every season for teleop
     * @param in The key for running the intake inwards
     * @param out The key for running the intake outwards
     * @param motor The motor to use
     * @param controller The controller to use
     */
    public void updateTwoWayMotorController(OptimizedController.Key in, OptimizedController.Key out, DcMotor motor, OptimizedController controller) {
        if(controller.getFloat(in) > 0) {
            motor.setPower(controller.getFloat(in));
        } else {
            motor.setPower(-controller.getFloat(out));
        }
    }

    /**
     * Simple two way motor controls -- generally this form of input is used every season for teleop
     * @param inControl The control map name for running the intake inwards
     * @param outControl The control map name for running the intake outwards
     * @param motor The motor to use
     * @param controller The controller to use
     */
    public void updateTwoWayMotorController(String inControl, String outControl, DcMotor motor, OptimizedController controller) {
        if(controller.getFloat(robot.getControl(inControl)) > 0) {
            motor.setPower(controller.getFloat(robot.getControl(inControl)));
        } else {
            motor.setPower(-controller.getFloat(robot.getControl(outControl)));
        }
    }

    /**
     * Simple two way motor controls -- generally this form of input is used every season for teleop
     * @param inControl The control map name for running the intake inwards
     * @param outControl The control map name for running the intake outwards
     * @param motors The motors to use
     * @param controller The controller to use
     */
    public void updateTwoWayMotorController(String inControl, String outControl, List<DcMotor> motors, OptimizedController controller) {
        if(controller.getFloat(robot.getControl(inControl)) > 0) {
            for(DcMotor motor : motors){
                motor.setPower(controller.getFloat(robot.getControl(inControl)));
            }
        } else {
            for(DcMotor motor : motors){
                motor.setPower(-controller.getFloat(robot.getControl(outControl)));
            }
        }
    }

    /**
     * Simple two way motor controls -- generally this form of input is used every season for teleop
     * @param in The key for running the intake inwards
     * @param out The key for running the intake outwards
     * @param motors The motors to use
     * @param controller The controller to use
     */
    public void updateTwoWayMotorController(OptimizedController.Key in, OptimizedController.Key out, List<DcMotor> motors, OptimizedController controller) {
        if(controller.getFloat(in) > 0) {
            for(DcMotor motor : motors){
                motor.setPower(controller.getFloat(in));
            }
        } else {
            for(DcMotor motor : motors){
                motor.setPower(-controller.getFloat(out));
            }
        }
    }

    /**
     * Simple two way motor controls -- generally this form of input is used every season for teleop
     * @param inControl The control map name for running the intake inwards
     * @param outControl The control map name for running the intake outwards
     * @param motors The motors to use
     * @param controller The controller to use
     * @param maxVelocity The maximum velocity to use
     */
    public void updateTwoWayMotorControllerVelocity(String inControl, String outControl, List<DcMotorEx> motors, OptimizedController controller, double maxVelocity) {
        if(controller.getFloat(robot.getControl(inControl)) > 0) {
            for(DcMotorEx motor : motors){
                motor.setVelocity(controller.getFloat(robot.getControl(inControl)) * maxVelocity);
            }
        } else {
            for(DcMotorEx motor : motors){
                motor.setVelocity(-controller.getFloat(robot.getControl(outControl)) * maxVelocity);
            }
        }
    }

    /**
     * Simple two way motor controls -- generally this form of input is used every season for teleop
     * @param in The key for running the intake inwards
     * @param out The key for running the intake outwards
     * @param motors The motors to use
     * @param controller The controller to use
     * @param maxVelocity The maximum velocity to use
     */
    public void updateTwoWayMotorControllerVelocity(OptimizedController.Key in, OptimizedController.Key out, List<DcMotorEx> motors, OptimizedController controller, double maxVelocity) {
        if(controller.getFloat(in) > 0) {
            for(DcMotorEx motor : motors){
                motor.setVelocity(controller.getFloat(in) * maxVelocity);
            }
        } else {
            for(DcMotorEx motor : motors){
                motor.setVelocity(-controller.getFloat(out) * maxVelocity);
            }
        }
    }

    /**
     * Simple two way motor controls -- generally this form of input is used every season for teleop
     * @param in The key for running the intake inwards
     * @param out The key for running the intake outwards
     * @param motor The motor to use
     * @param controller The controller to use
     * @param maxVelocity The maximum velocity to use
     */
    public void updateTwoWayMotorControllerVelocity(OptimizedController.Key in, OptimizedController.Key out, DcMotorEx motor, OptimizedController controller, double maxVelocity) {
        if(controller.getFloat(in) > 0) {
            motor.setVelocity(controller.getFloat(in) * maxVelocity);
        } else {
            motor.setVelocity(controller.getFloat(out) * maxVelocity);
        }
    }

    /**
     * Simple two way motor controls -- generally this form of input is used every season for teleop
     * @param inControl The control map name for running the intake inwards
     * @param outControl The control map name for running the intake outwards
     * @param motor The motor to use
     * @param controller The controller to use
     * @param maxVelocity The maximum velocity to use
     */
    public void updateTwoWayMotorControllerVelocity(String inControl, String outControl, DcMotorEx motor, OptimizedController controller, double maxVelocity) {
        if(controller.getFloat(robot.getControl(inControl)) > 0) {
            motor.setVelocity(controller.getFloat(robot.getControl(inControl)) * maxVelocity);
        } else {
            motor.setVelocity(controller.getFloat(robot.getControl(outControl)) * maxVelocity);
        }
    }

    /**
     * Simple two way motor controls -- generally this form of input is used every season for teleop
     * @param inControl The control map name for running the intake inwards
     * @param outControl The control map name for running the intake outwards
     * @param motors The motors to use
     * @param controller The controller to use
     * @param maxVelocity The maximum velocity to use
     * @param velocityUnit The units to use for velocity
     */
    public void updateTwoWayMotorControllerVelocity(String inControl, String outControl, List<DcMotorEx> motors, OptimizedController controller, double maxVelocity, AngleUnit velocityUnit) {
        if(controller.getFloat(robot.getControl(inControl)) > 0) {
            for(DcMotorEx motor : motors){
                motor.setVelocity(controller.getFloat(robot.getControl(inControl)) * maxVelocity, velocityUnit);
            }
        } else {
            for(DcMotorEx motor : motors){
                motor.setVelocity(-controller.getFloat(robot.getControl(outControl)) * maxVelocity, velocityUnit);
            }
        }
    }

    /**
     * Simple two way motor controls -- generally this form of input is used every season for teleop
     * @param in The key for running the intake inwards
     * @param out The key for running the intake outwards
     * @param motors The motors to use
     * @param controller The controller to use
     * @param maxVelocity The maximum velocity to use
     * @param velocityUnit The units to use for velocity
     */
    public void updateTwoWayMotorControllerVelocity(OptimizedController.Key in, OptimizedController.Key out, List<DcMotorEx> motors, OptimizedController controller, double maxVelocity, AngleUnit velocityUnit) {
        if(controller.getFloat(in) > 0) {
            for(DcMotorEx motor : motors){
                motor.setVelocity(controller.getFloat(in) * maxVelocity, velocityUnit);
            }
        } else {
            for(DcMotorEx motor : motors){
                motor.setVelocity(-controller.getFloat(out) * maxVelocity, velocityUnit);
            }
        }
    }

    /**
     * Simple two way motor controls -- generally this form of input is used every season for teleop
     * @param in The key for running the intake inwards
     * @param out The key for running the intake outwards
     * @param motor The motor to use
     * @param controller The controller to use
     * @param maxVelocity The maximum velocity to use
     * @param velocityUnit The units to use for velocity
     */
    public void updateTwoWayMotorControllerVelocity(OptimizedController.Key in, OptimizedController.Key out, DcMotorEx motor, OptimizedController controller, double maxVelocity, AngleUnit velocityUnit) {
        if(controller.getFloat(in) > 0) {
            motor.setVelocity(controller.getFloat(in) * maxVelocity, velocityUnit);
        } else {
            motor.setVelocity(controller.getFloat(out) * maxVelocity, velocityUnit);
        }
    }

    /**
     * Simple two way motor controls -- generally this form of input is used every season for teleop
     * @param inControl The control map name for running the intake inwards
     * @param outControl The control map name for running the intake outwards
     * @param motor The motor to use
     * @param controller The controller to use
     * @param maxVelocity The maximum velocity to use
     * @param velocityUnit The units to use for velocity
     */
    public void updateTwoWayMotorControllerVelocity(String inControl, String outControl, DcMotorEx motor, OptimizedController controller, double maxVelocity, AngleUnit velocityUnit) {
        if(controller.getFloat(robot.getControl(inControl)) > 0) {
            motor.setVelocity(controller.getFloat(robot.getControl(inControl)) * maxVelocity, velocityUnit);
        } else {
            motor.setVelocity(controller.getFloat(robot.getControl(outControl)) * maxVelocity, velocityUnit);
        }
    }

    /**
     * Stops all four drive motors
     */
    public void stopMotors() {
        robot.motors[0].setPower(0);
        robot.motors[1].setPower(0);
        robot.motors[2].setPower(0);
        robot.motors[3].setPower(0);
    }

    /**
     * Runs certain distance based on built-in drive motor encoders
     * @param distanceCM Distance to travel in centimeters
     * @param power The power to run at
     */
    @OldCode
    public void runToPos(double distanceCM, float power){

        if(power < 0){
            power *= -1;
            robot.motorDir(false);
        }

        double revs = distanceCM/robot.CIRCUMFERENCE;
        int ticksToRun = (int)(revs * RobotConfig.ENCODER_TICK_PER_REV);
        robot.runWithEncoder(true);
        robot.motors[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motors[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motors[2].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motors[3].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motors[0].setTargetPosition(ticksToRun);
        robot.motors[1].setTargetPosition(ticksToRun);
        robot.motors[2].setTargetPosition(ticksToRun);
        robot.motors[3].setTargetPosition(ticksToRun);
        linearDrive(power);
        while (robot.motors[0].isBusy() || robot.motors[1].isBusy() || robot.motors[2].isBusy() || robot.motors[3].isBusy()){
            //This line was intentionally left blank
        }
        stopMotors();
        robot.runWithEncoder(false);
        robot.motorDir(true);
    }

    /**
     * Simple teleop function to change some variable using two keys by an increment -- this function will auto cap the val between two bounds
     * This method is useful for calibration opmodes to change some motor power, servo position, etc.
     * @param initialValue The current value you want to change
     * @param maxValue The maximum the value can go
     * @param minValue The minimum the value can go
     * @param increaseKey The key to increase
     * @param decreaseKey The key to decrease
     * @param controller The controller to use
     * @param increment The increment value
     * @return The incremented value
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    @Experimental
    public double modifyValueByIncrement(double initialValue, double maxValue, double minValue, OptimizedController.Key increaseKey, OptimizedController.Key decreaseKey, @NotNull OptimizedController controller, double increment) {
        increment = Math.abs(increment);
        if (controller.getOnPress(decreaseKey)) {
            return ((initialValue - increment) % maxValue) + minValue;
        } else if(controller.getOnPress(increaseKey)) {
            return ((initialValue + increment) % maxValue) + minValue;
        } else {
            return initialValue;
        }
    }

    /**
     * Just very simple code to toggle a servo between two positions -- Just trying to make coding more modular
     * @param servo The servo you want to use
     * @param initialPos The position you want the servo to assume on start
     * @param otherPos The "other" position for the servo to go to after pressing the key
     * @param controller The controller to use
     * @param key The key on the controller to toggle
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    public void toggleServoBetweenPos(Servo servo, double initialPos, double otherPos, OptimizedController controller, OptimizedController.Key key) {
        if(controller.getToggle(key))
            servo.setPosition(otherPos);
        else
            servo.setPosition(initialPos);
    }
}
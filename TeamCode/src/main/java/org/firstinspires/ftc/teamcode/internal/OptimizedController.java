package org.firstinspires.ftc.teamcode.internal;

import android.os.Build;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.HashMap;

import androidx.annotation.RequiresApi;

/**
 * A class that adds a bunch of extra functionality to a normal gamepad, such as toggle buttons and onPress events.
 * Access these custom controllers through an instance of the OptimizedRobot class!
 *
 * @author Owen Boseley - Class of 2021
 */
public class OptimizedController {

    /**
     * Internal gamepad
     */
    private Gamepad internalGamepad = null;

    /**
     * The two gamepads
     */
    private Gamepad gamepad1 = null, gamepad2 = null;

    /**
     * Our start key
     */
    private Key startKey = null;

    /**
     * Required gamepad
     */
    private Gamepad requiredGamepad = null;

    /**
     * Hashmaps used in storing internal boolean states for our gamepad keys
     */
    private HashMap<Key, Boolean> canToggleList = new HashMap<>();
    private HashMap<Key, Boolean> toggledList = new HashMap<>();
    private HashMap<Key, Boolean> beforeStateList = new HashMap<>();

    /**
     * All the keys that are currently disabled on this controller
     */
    private ArrayList<Key> keysDisabled = new ArrayList<Key>();

    /**
     * Constructor
     * @param requiredGamepad If assigned a value, on that gamepad can use this controller -- null if you want this to be optional
     */
    protected OptimizedController(Gamepad requiredGamepad, Key startKey, Gamepad gamepad1, Gamepad gamepad2){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.requiredGamepad = requiredGamepad;
        this.startKey = startKey;
        {
            canToggleList.put(Key.START, true);
            canToggleList.put(Key.A, true);
            canToggleList.put(Key.B, true);
            canToggleList.put(Key.X, true);
            canToggleList.put(Key.Y, true);
            canToggleList.put(Key.LEFT_BUMPER, true);
            canToggleList.put(Key.RIGHT_BUMPER, true);
            canToggleList.put(Key.LEFT_TRIGGER, true);
            canToggleList.put(Key.RIGHT_TRIGGER, true);
            canToggleList.put(Key.LEFT_STICK_X, true);
            canToggleList.put(Key.LEFT_STICK_Y, true);
            canToggleList.put(Key.RIGHT_STICK_X, true);
            canToggleList.put(Key.RIGHT_STICK_Y, true);
            canToggleList.put(Key.DPAD_UP, true);
            canToggleList.put(Key.DPAD_DOWN, true);
            canToggleList.put(Key.DPAD_LEFT, true);
            canToggleList.put(Key.DPAD_RIGHT, true);
            canToggleList.put(Key.BACK, true);
        }
        {
            toggledList.put(Key.START, false);
            toggledList.put(Key.A, false);
            toggledList.put(Key.B, false);
            toggledList.put(Key.X, false);
            toggledList.put(Key.Y, false);
            toggledList.put(Key.LEFT_BUMPER, false);
            toggledList.put(Key.RIGHT_BUMPER, false);
            toggledList.put(Key.LEFT_TRIGGER, false);
            toggledList.put(Key.RIGHT_TRIGGER, false);
            toggledList.put(Key.LEFT_STICK_X, false);
            toggledList.put(Key.LEFT_STICK_Y, false);
            toggledList.put(Key.RIGHT_STICK_X, false);
            toggledList.put(Key.RIGHT_STICK_Y, false);
            toggledList.put(Key.DPAD_UP, false);
            toggledList.put(Key.DPAD_DOWN, false);
            toggledList.put(Key.DPAD_LEFT, false);
            toggledList.put(Key.DPAD_RIGHT, false);
            toggledList.put(Key.BACK, false);
        }
        {
            beforeStateList.put(Key.START, false);
            beforeStateList.put(Key.A, false);
            beforeStateList.put(Key.B, false);
            beforeStateList.put(Key.X, false);
            beforeStateList.put(Key.Y, false);
            beforeStateList.put(Key.LEFT_BUMPER, false);
            beforeStateList.put(Key.RIGHT_BUMPER, false);
            beforeStateList.put(Key.LEFT_TRIGGER, false);
            beforeStateList.put(Key.RIGHT_TRIGGER, false);
            beforeStateList.put(Key.LEFT_STICK_X, false);
            beforeStateList.put(Key.LEFT_STICK_Y, false);
            beforeStateList.put(Key.RIGHT_STICK_X, false);
            beforeStateList.put(Key.RIGHT_STICK_Y, false);
            beforeStateList.put(Key.DPAD_UP, false);
            beforeStateList.put(Key.DPAD_DOWN, false);
            beforeStateList.put(Key.DPAD_LEFT, false);
            beforeStateList.put(Key.DPAD_RIGHT, false);
            beforeStateList.put(Key.BACK, false);
        }
    }

    /**
     * The keys on the controller
     */
    public enum Key {
        START, A, B, X, Y, LEFT_BUMPER, RIGHT_BUMPER, LEFT_TRIGGER, RIGHT_TRIGGER, LEFT_STICK_X, LEFT_STICK_Y, RIGHT_STICK_X, RIGHT_STICK_Y, DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT, BACK;
    }

    /**
     * Tells whether this controller's sticks are being used
     * @return a boolean on whether or not this controller is being used
     */
    public boolean isBeingUsed() {
        return (!NumberFunctions.isZero(getFloat(Key.LEFT_STICK_Y)) || !NumberFunctions.isZero(getFloat(Key.LEFT_STICK_X)) || !NumberFunctions.isZero(getFloat(Key.RIGHT_STICK_X)) || !NumberFunctions.isZero(getFloat(Key.RIGHT_STICK_Y)));
    }

    /**
     * Disable a key on the controller
     * @param key The key
     * @param isDisabled Whether or not you want this key to be disabled
     */
    public void disableKey(Key key, boolean isDisabled) {
        if(isDisabled)
            keysDisabled.add(key);
        else
            keysDisabled.remove(key);
    }

    /**
     * Returns whether the inputted key is disabled
     * @param key The key to use
     * @return The state of disabled
     */
    private boolean isDisabled(Key key) {
        return keysDisabled.contains(key) ? true : false;
    }

    /**
     * Changes the gamepad associated with this controller
     */
    protected void setInternalGamepad(Gamepad gamepad) {
       this.internalGamepad = gamepad;
    }

    /**
     * Will automatically search to see if a gamepad has correctly pressed the buttons to activate this virtual controller
     */
    protected void updateInternalGamepad() {
        if(gamepad1.start && getBool(startKey, gamepad1) && (requiredGamepad.equals(gamepad1) || requiredGamepad == null)) {
            internalGamepad = gamepad1;
        } else if(gamepad2.start && getBool(startKey, gamepad2) && (requiredGamepad.equals(gamepad2) || requiredGamepad == null)) {
            internalGamepad = gamepad2;
        }
    }


    /**
     * Returns whether this key is traditionally a key that returns a number value associated with it
     * @param key The key to use
     */
    public static boolean isFloatingTypeKey(Key key) {
        return key == Key.RIGHT_STICK_X || key == Key.RIGHT_STICK_Y || key == Key.LEFT_STICK_X || key == Key.LEFT_STICK_Y || key == Key.LEFT_TRIGGER || key == Key.RIGHT_TRIGGER;
    }

    /**
     * Toggles stored boolean value, for this key, when specified key is pressed
     * @param key The key to use
     * @return The state of the toggle
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    public boolean getToggle(Key key) {
        if(internalGamepad == null)
            return false;
        if(isDisabled(key))
            return false;
        for(Key k : canToggleList.keySet()) {
            if(key == k) {
                if(getBool(key) && canToggleList.get(key)){
                    toggledList.replace(key, toggledList.get(key) ? false : true);
                    canToggleList.replace(key, false);
                } else if(!getBool(key)){
                    canToggleList.replace(key, true);
                }
                return toggledList.get(key);
            }
        }
        return false;
    }

    /**
     * Gets the boolean value of a key
     * @param key The key to use
     * @return A boolean on whether or not this key is being pressed down
     */
    public boolean getBool(Key key){
        if(internalGamepad == null)
            return false;
        if(isDisabled(key))
            return false;

        if(key == Key.A){
            return internalGamepad.a;
        } else if(key == Key.B){
            return internalGamepad.b;
        } else if(key == Key.X){
            return internalGamepad.x;
        } else if(key == Key.Y){
            return internalGamepad.y;
        } else if(key == Key.LEFT_BUMPER){
            return internalGamepad.left_bumper;
        } else if(key == Key.RIGHT_BUMPER){
            return internalGamepad.right_bumper;
        } else if(key == Key.DPAD_UP){
            return internalGamepad.dpad_up;
        } else if(key == Key.DPAD_DOWN){
            return internalGamepad.dpad_down;
        } else if(key == Key.DPAD_LEFT){
            return internalGamepad.dpad_left;
        } else if(key == Key.DPAD_RIGHT){
            return internalGamepad.dpad_right;
        } else if(key == Key.START){
            return internalGamepad.start;
        } else if(key == Key.BACK){
            return internalGamepad.back;
        } else if(key == Key.LEFT_STICK_X){
            return !NumberFunctions.isZero(internalGamepad.left_stick_x);
        } else if(key == Key.LEFT_STICK_Y){
            return !NumberFunctions.isZero(internalGamepad.left_stick_y);
        } else if(key == Key.RIGHT_STICK_X){
            return !NumberFunctions.isZero(internalGamepad.right_stick_x);
        } else if(key == Key.RIGHT_STICK_Y){
            return !NumberFunctions.isZero(internalGamepad.right_stick_y);
        } else if(key == Key.LEFT_TRIGGER){
            return !NumberFunctions.isZero(internalGamepad.left_trigger);
        } else if(key == Key.RIGHT_TRIGGER){
            return !NumberFunctions.isZero(internalGamepad.right_trigger);
        } else {
            return false;
        }
    }

    /**
     * Gets the boolean value of a key for a specific controller
     * @param key The key to use
     * @return A boolean on whether or not this key is being pressed down
     */
    private static boolean getBool(Key key, Gamepad gamepad){
        if(key == Key.A){
            return gamepad.a;
        } else if(key == Key.B){
            return gamepad.b;
        } else if(key == Key.X){
            return gamepad.x;
        } else if(key == Key.Y){
            return gamepad.y;
        } else if(key == Key.LEFT_BUMPER){
            return gamepad.left_bumper;
        } else if(key == Key.RIGHT_BUMPER){
            return gamepad.right_bumper;
        } else if(key == Key.DPAD_UP){
            return gamepad.dpad_up;
        } else if(key == Key.DPAD_DOWN){
            return gamepad.dpad_down;
        } else if(key == Key.DPAD_LEFT){
            return gamepad.dpad_left;
        } else if(key == Key.DPAD_RIGHT){
            return gamepad.dpad_right;
        } else if(key == Key.START){
            return gamepad.start;
        } else if(key == Key.BACK){
            return gamepad.back;
        } else if(key == Key.LEFT_STICK_X){
            return !NumberFunctions.isZero(gamepad.left_stick_x);
        } else if(key == Key.LEFT_STICK_Y){
            return !NumberFunctions.isZero(gamepad.left_stick_y);
        } else if(key == Key.RIGHT_STICK_X){
            return !NumberFunctions.isZero(gamepad.right_stick_x);
        } else if(key == Key.RIGHT_STICK_Y){
            return !NumberFunctions.isZero(gamepad.right_stick_y);
        } else if(key == Key.LEFT_TRIGGER){
            return !NumberFunctions.isZero(gamepad.left_trigger);
        } else if(key == Key.RIGHT_TRIGGER){
            return !NumberFunctions.isZero(gamepad.right_trigger);
        } else {
            return false;
        }
    }

    /**
     * Returns true for one frame when specified key is pressed
     * @param key The key is use
     * @return The state
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    public boolean getOnPress(Key key) {
        if(internalGamepad == null)
            return false;
        if(isDisabled(key))
            return false;

        for(Key k : beforeStateList.keySet()) {
            if(k == key) {
                if(!beforeStateList.get(key) && getBool(key)) {
                    return beforeStateList.replace(key, true);
                }else{
                    beforeStateList.replace(key, getBool(key));
                    return false;
                }
            }
        }
        return false;
    }

    /**
     * Returns true for one frame when specified key is released
     * @param key The key is use
     * @return The state
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    public boolean getOnRelease(Key key) {
        if(internalGamepad == null)
            return false;
        if(isDisabled(key))
            return false;

        for(Key k : beforeStateList.keySet()) {
            if(k == key) {
                if(beforeStateList.get(key) && !getBool(key)) {
                    beforeStateList.replace(key, false);
                    return true;
                }else{
                    beforeStateList.replace(key, getBool(key));
                    return false;
                }
            }
        }
        return false;
    }

    /**
     * Gets the float value of keys on the controller (such as triggers or sticks)
     * @param key The key to use
     * @return The float value returned from this key
     */
    public float getFloat(Key key){
        if(internalGamepad == null)
            return 0;
        if(isDisabled(key))
            return 0;

        if(key == Key.LEFT_STICK_X){
            return internalGamepad.left_stick_x;
        } else if(key == Key.LEFT_STICK_Y){
            return internalGamepad.left_stick_y;
        } else if(key == Key.RIGHT_STICK_X){
            return internalGamepad.right_stick_x;
        } else if(key == Key.RIGHT_STICK_Y){
            return internalGamepad.right_stick_y;
        } else if(key == Key.LEFT_TRIGGER){
            return internalGamepad.left_trigger;
        } else if(key == Key.RIGHT_TRIGGER) {
            return internalGamepad.right_trigger;
        } else if(key == Key.A){
            return internalGamepad.a ? 1 : 0;
        } else if(key == Key.B){
            return internalGamepad.b ? 1 : 0;
        } else if(key == Key.X){
            return internalGamepad.x ? 1 : 0;
        } else if(key == Key.Y){
            return internalGamepad.y ? 1 : 0;
        } else if(key == Key.LEFT_BUMPER){
            return internalGamepad.left_bumper ? 1 : 0;
        } else if(key == Key.RIGHT_BUMPER){
            return internalGamepad.right_bumper ? 1 : 0;
        } else if(key == Key.DPAD_UP){
            return internalGamepad.dpad_up ? 1 : 0;
        } else if(key == Key.DPAD_DOWN){
            return internalGamepad.dpad_down ? 1 : 0;
        } else if(key == Key.DPAD_LEFT){
            return internalGamepad.dpad_left ? 1 : 0;
        } else if(key == Key.DPAD_RIGHT){
            return internalGamepad.dpad_right ? 1 : 0;
        } else if(key == Key.START){
            return internalGamepad.start ? 1 : 0;
        } else if(key == Key.BACK){
            return internalGamepad.back ? 1 : 0;
        } else {
            return 0;
        }
    }
}
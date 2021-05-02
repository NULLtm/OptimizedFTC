package org.firstinspires.ftc.teamcode.internal;

public class NumberFunctions {


    private static final double DOUBLE_TOLERANCE = 0.001;


    /**
     * Approximates a float around zero using a tolerance level
     */
    public static boolean isZero(double db) {
        return Math.abs(db) < DOUBLE_TOLERANCE;
    }
}

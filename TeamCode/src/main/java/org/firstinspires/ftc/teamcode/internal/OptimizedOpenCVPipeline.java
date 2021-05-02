package org.firstinspires.ftc.teamcode.internal;

import org.firstinspires.ftc.teamcode.internal.OpenCVBuilder;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * This is just an extension of the OpenCVPipline class which adds support for vision output to the OptimizedRobot Class
 */
public abstract class OptimizedOpenCVPipeline extends OpenCvPipeline {

    /**
     * Our output for vision -- TODO: set this value in your Pipeline
     */
    protected String outputToken;

    /**
     * Internal method to get this output
     * @return The output token
     */
    public String getVisionOutput() {
        return outputToken;
    }

    /**
     * The normal processing method
     * @param input The input image from the camera
     * @return The image to show to the phone
     */
    public abstract Mat processFrame(Mat input);
}

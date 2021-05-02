package org.firstinspires.ftc.teamcode.internal;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;

/**
 * A useful assistant in the creation of EasyOpenCV vision pipelines
 * @author Owen Boseley - Class of 2021
 */
public class OpenCVBuilder {

    // The original input image -- has no color space change
    private Mat inputImg = null;

    // Color space changed image
    private Mat newInputImg = null;

    // Our singleton instance
    private static OpenCVBuilder instance = null;

    // The submats we are using in the pipeline
    private HashMap<String, Mat> resources = new HashMap<String, Mat>();

    // The bounds of created submats to be used for outlines or labels later on
    private HashMap<String, int[]> resourceDimensions = new HashMap<String, int[]>();


    /**
     * Our constructor
     * @param inputImg the input var coming from the camera (the input of the
     */
    private OpenCVBuilder(Mat inputImg) {
        this.inputImg = inputImg;
        this.newInputImg = inputImg;
    }


    /**
     * This class is built as a singleton -- only one instance exists
     * Instead of instantiating this class, you will use this method to call the instance
     * @param input the input image from the OpenCV pipeline
     * @return The singleton
     */
    public static OpenCVBuilder getInstance(Mat input) {
        if(instance == null) {
            instance = new OpenCVBuilder(input);
        } else {
            instance.inputImg = input;
        }
        return instance;
    }

    /**
     * Using a coordinate system of proportions of the image based on (0,0) at the center,
     * this method will return a submat of the modified input image (each bound is from -1.0 to 1.0)
     * @return The newly created submat of the modified input image
     * @param matName The name to assign to this partitioned submat
     */
    public void createSubmat(String matName, double leftBound, double rightBound, double lowerBound, double upperBound) {
        // Our center
        int centerX = inputImg.cols() / 2, centerY = inputImg.rows() / 2;

        int startX = (int) ((1 - leftBound) * centerX);
        int endX = (int) ((1 + rightBound) * centerX);

        int startY = (int) ((1 - upperBound) * centerY);
        int endY = (int) ((1 + lowerBound) * centerY);

        Mat submat = newInputImg.submat(startY, endY, startX, endX);
        resources.put(matName, submat);
        int[] dimensions = {startX, startY, endX, endY};
        resourceDimensions.put(matName, dimensions);
    }

    /**
     * Using a coordinate system of proportions of the image based on (0,0) at the center,
     * this method will return a submat of the modified input image (each bound is from -1.0 to 1.0)
     * @return The newly created submat of the modified input image
     * @param matName The name to assign to this partitioned submat
     * @param createOutline Whether or not to automatically render a rectangle for the input image to visualize what this submat looks like
     */
    @Experimental
    public void createSubmat(String matName, double leftBound, double rightBound, double lowerBound, double upperBound, boolean createOutline) {
        // Our center
        int centerX = inputImg.cols() / 2, centerY = inputImg.rows() / 2;

        int startX = (int) ((1 - leftBound) * centerX);
        int endX = (int) ((1 + rightBound) * centerX);

        int startY = (int) ((1 - upperBound) * centerY);
        int endY = (int) ((1 + lowerBound) * centerY);

        Mat submat = newInputImg.submat(startY, endY, startX, endX);

        if(createOutline) {
            Imgproc.rectangle(inputImg, new Point(startX, startY), new Point(endX, endY), new Scalar(230, 100, 50), 3);
            Imgproc.rectangle(newInputImg, new Point(startX, startY), new Point(endX, endY), new Scalar(230, 100, 50), 3);
        }

        resources.put(matName, submat);
    }


    /**
     * This grabs the mean (average) value of the specified color type (in HSV, for example, 0 = H, 1 = S, 2 = V) of the specified submat
     * @param submatName The name of the submat
     * @param colorID The element of the color space to use (starts counting at 0)
     * @return The average value
     */
    @Repackaged
    public double getMeanColor(String submatName, int colorID) {
        return Core.mean(resources.get(submatName)).val[colorID];
    }

    /**
     * This grabs the mean (average) value of the specified color type (in HSV, for example, 0 = H, 1 = S, 2 = V) for the modified input image
     * @param colorID The element of the color space to use (starts counting at 0)
     * @return The average value
     */
    @Repackaged
    public double getMeanColor(int colorID) {
        return Core.mean(resources.get(newInputImg)).val[colorID];
    }


    /**
     * Renders an outline for the phone to see of a specified submat
     * @param submatName The submat to outline
     * @param applyToOriginalInput Whether or not to apply this outline to the modified color-space input image or the ORIGINAL coming from the camera
     */
    public void outlineSubmat(String submatName, boolean applyToOriginalInput) {
        int[] dimensions = resourceDimensions.get(submatName);

        if(applyToOriginalInput)
            Imgproc.rectangle(inputImg, new Point(dimensions[0], dimensions[1]), new Point(dimensions[2], dimensions[3]), new Scalar(230, 100, 50), 3);
        else
            Imgproc.rectangle(newInputImg, new Point(dimensions[0], dimensions[1]), new Point(dimensions[2], dimensions[3]), new Scalar(230, 100, 50), 3);
    }


    /**
     * Creates a label on the specified input mat for a specified submat
     * @param submatName The name of the submat to label
     * @param labelText The text on the label
     * @param xOffset The X offset from the top-left corner of the submat
     * @param yOffset The Y offset from the top-left corner of the submat (inverted, mind you)
     * @param fontSize The font size of the label (2 is a pretty normal size, for reference)
     * @param applyToOriginalInput Whether or not to apply this outline to the modified color-space input image or the ORIGINAL coming from the camera
     */
    public void labelSubmat(String submatName, String labelText, int xOffset, int yOffset, int fontSize, boolean applyToOriginalInput) {
        int[] dimensions = resourceDimensions.get(submatName);

        int x = dimensions[0] + xOffset, y = dimensions[1] + yOffset;

        if(applyToOriginalInput)
            Imgproc.putText(inputImg, labelText, new Point(x, y), 1, fontSize, new Scalar(20, 100, 0), 2);
        else
            Imgproc.putText(newInputImg, labelText, new Point(x, y), 1, fontSize, new Scalar(20, 100, 0), 2);
    }

    /**
     * Creates a label on the specified input mat for a specified submat
     * @param submatName The name of the submat to label
     * @param labelText The text on the label
     * @param applyToOriginalInput Whether or not to apply this outline to the modified color-space input image or the ORIGINAL coming from the camera
     */
    public void labelSubmat(String submatName, String labelText, boolean applyToOriginalInput) {
        int[] dimensions = resourceDimensions.get(submatName);

        int x = dimensions[0], y = dimensions[1];

        if(applyToOriginalInput)
            Imgproc.putText(inputImg, labelText, new Point(x, y), 1, 1, new Scalar(20, 100, 0), 2);
        else
            Imgproc.putText(newInputImg, labelText, new Point(x, y), 1, 1, new Scalar(20, 100, 0), 2);
    }

    /**
     * Creates a label on the specified input mat for a specified submat
     * @param submatName The name of the submat to label
     * @param labelText The text on the label
     * @param xOffset The X offset from the top-left corner of the submat
     * @param yOffset The Y offset from the top-left corner of the submat (inverted, mind you)
     * @param applyToOriginalInput Whether or not to apply this outline to the modified color-space input image or the ORIGINAL coming from the camera
     */
    public void labelSubmat(String submatName, String labelText, int xOffset, int yOffset, boolean applyToOriginalInput) {
        int[] dimensions = resourceDimensions.get(submatName);

        int x = dimensions[0] + xOffset, y = dimensions[1] + yOffset;

        if(applyToOriginalInput)
            Imgproc.putText(inputImg, labelText, new Point(x, y), 1, 1, new Scalar(20, 100, 0), 2);
        else
            Imgproc.putText(newInputImg, labelText, new Point(x, y), 1, 1, new Scalar(20, 100, 0), 2);
    }

    /**
     * Changes the color space of the camera input image
     * @param colorSpace the color space to change to
     * @see Imgproc
     */
    public void changeColorSpace(int colorSpace) {
        Mat outputImg = new Mat();
        Imgproc.cvtColor(inputImg, outputImg, colorSpace);
        newInputImg = outputImg;
    }

    /**
     * Changes the color space of the inputted image
     * @param colorSpace the color space to change to
     * @return the material with the new color space
     * @param input the material to use
     * @see Imgproc
     */
    @Deprecated
    public Mat changeColorSpace(Mat input, int colorSpace) {
        Mat outputImg = new Mat();
        Imgproc.cvtColor(input, outputImg, colorSpace);
        return outputImg;
    }


    /**
     * Releases all Mat resources used during the pipeline
     * Must be called right before the end of the processFrame() method in your pipeline
     */
    public void releaseResources() {
        for(String key : resources.keySet()) {
            resourceDimensions.remove(key);
        }

        Iterator<Mat> it = resources.values().iterator();

        while(it.hasNext()) {
            Mat n = it.next();
            n.release();
            it.remove();
        }
    }


    /**
     * Returns the input image so that it can be displayed on the phone
     * Also releases the resources of the input image not being used -- SO BE CAREFUL
     * @param applyChanges Whether or not to use the color space changed input or not
     * @return The camera image
     */
    public Mat getInputImage(boolean applyChanges) {
        if(applyChanges) {
            inputImg.release();
            return newInputImg;
        } else {
            newInputImg.release();
            return inputImg;
        }
    }


    /**
     * Outlines all created submats
     * @param applyToOriginalInput Whether or not to apply these outlines to the original input or the
     * changed color space input.
     */
    public void outlineSubmats(boolean applyToOriginalInput) {
        for(String key : resources.keySet()) {
            int[] dimensions = resourceDimensions.get(key);

            if(applyToOriginalInput)
                Imgproc.rectangle(inputImg, new Point(dimensions[0], dimensions[1]), new Point(dimensions[2], dimensions[3]), new Scalar(230, 100, 50), 3);
            else
                Imgproc.rectangle(newInputImg, new Point(dimensions[0], dimensions[1]), new Point(dimensions[2], dimensions[3]), new Scalar(230, 100, 50), 3);
        }
    }

}

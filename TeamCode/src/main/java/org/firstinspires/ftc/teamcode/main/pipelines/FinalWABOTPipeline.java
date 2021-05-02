package org.firstinspires.ftc.teamcode.main.pipelines;

import org.firstinspires.ftc.teamcode.internal.OpenCVBuilder;
import org.firstinspires.ftc.teamcode.internal.OptimizedOpenCVPipeline;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class FinalWABOTPipeline extends OptimizedOpenCVPipeline {

    private OpenCVBuilder builder;

    @Override
    public Mat processFrame(Mat input) {

        builder = OpenCVBuilder.getInstance(input);

        builder.changeColorSpace(Imgproc.COLOR_RGB2HSV);

        builder.createSubmat("Top", 0.3, 0.7, -0.3, 0.55);
        builder.createSubmat("Bottom", 0.3, 0.7, 0.5, -0.15);

        double hue1 = builder.getMeanColor("Top", 2);
        double hue2 = builder.getMeanColor("Bottom", 2);

        if(hue1 > 150)
            outputToken = "4";
        else if(hue2 > 175)
            outputToken = "1";
        else
            outputToken = "0";

        builder.releaseResources();

        return builder.getInputImage(false);
    }
}

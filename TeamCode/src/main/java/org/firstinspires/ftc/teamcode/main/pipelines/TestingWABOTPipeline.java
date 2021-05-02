package org.firstinspires.ftc.teamcode.main.pipelines;

import org.firstinspires.ftc.teamcode.internal.OpenCVBuilder;
import org.firstinspires.ftc.teamcode.internal.OptimizedOpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TestingWABOTPipeline extends OptimizedOpenCVPipeline {

    private OpenCVBuilder builder;

    @Override
    public Mat processFrame(Mat input) {

        builder = OpenCVBuilder.getInstance(input);

        builder.changeColorSpace(Imgproc.COLOR_RGB2HSV);

        builder.createSubmat("Top", 0.3, 0.7, -0.3, 0.55);
        builder.createSubmat("Bottom", 0.3, 0.7, 0.5, -0.15);

        double hue1 = builder.getMeanColor("Top", 2);
        double hue2 = builder.getMeanColor("Bottom", 2);

        outputToken = "Top Hue " + (int)hue1 + "Bottom Hue" + (int)hue2;

        builder.labelSubmat("Top", outputToken, -90, 80, true);
        if(hue1 > 160)
            builder.labelSubmat("Top", "4R", -30, 50, true);
        else if(hue2 > 140)
            builder.labelSubmat("Top", "1R", -30, 50, true);
        else
            builder.labelSubmat("Top", "0R", -30, 50, true);

        builder.outlineSubmats(true);

        builder.releaseResources();

        return builder.getInputImage(false);
    }
}

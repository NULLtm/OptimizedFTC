package org.firstinspires.ftc.teamcode.main.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class OLDWABOTPipeline extends OpenCvPipeline {

    // Our mats
    private Mat convert = new Mat(), rect1 = null, rect2 = null;

    protected static int output = 0;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, convert, Imgproc.COLOR_RGB2HSV);

        int colStart = input.cols() / 2 - 80;
        int colEnd = input.cols() / 2 + 80;
        int rowStart1 = 75;
        int rowEnd1 = 100;
        int rowStart2 = input.rows() - 60;
        int rowEnd2 = input.rows() - 40;

        rect1 = convert.submat(rowStart1, rowEnd1, colStart, colEnd);
        rect2 = convert.submat(rowStart2, rowEnd2, colStart, colEnd);

        double hue1 = Core.mean(rect1).val[0];
        double hue2 = Core.mean(rect2).val[0];

        double idealHue = 10;
        double hueThreshold = 3;

        output = (int) hue1;

        if(Math.abs(idealHue - hue1) < hueThreshold)
            Imgproc.putText(input, "4 Rings Found", new Point(input.cols()/2, input.rows()/2), 1, 2, new Scalar(20, 100, 0), 2);
        else if(Math.abs(idealHue - hue2) < hueThreshold)
            Imgproc.putText(input, "1 Rings Found", new Point(input.cols()/2, input.rows()/2), 1, 2, new Scalar(20, 100, 0), 2);
        else
            Imgproc.putText(input, "0 Rings Found", new Point(input.cols()/2, input.rows()/2), 1, 2, new Scalar(20, 100, 0), 2);


        Imgproc.rectangle(input, new Point(colStart, rowStart1), new Point(colEnd, rowEnd1), new Scalar(230, 100, 50), 3);
        Imgproc.rectangle(input, new Point(colStart, rowStart2), new Point(colEnd, rowEnd2), new Scalar(230, 100, 50), 3);

        rect1.release();
        rect2.release();
        convert.release();

        return input;
    }

    protected int getOutput(){
        return output;
    }
}

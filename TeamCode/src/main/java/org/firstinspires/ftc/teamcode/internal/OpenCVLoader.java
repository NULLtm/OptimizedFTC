package org.firstinspires.ftc.teamcode.internal;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Locale;

/**
 * Internal class tasked with loader the openCV pipelines and other backend tasks using a WEBCAM for input
 * You probably won't be touching this unless some update comes out...
 */
public class OpenCVLoader {

    /**
     * The phone camera
     */
    private OpenCvCamera phoneCam;

    /**
     * The pipeline that is being used
     */
    protected OptimizedOpenCVPipeline pipeline;

    /**
     * The internal hardware map being used
     */
    private HardwareMap map;

    /**
     * Whether or not to show a view of the camera to the driver station phone
     */
    private final boolean RUN_ON_APP;

    /**
     * Our constructor
     * @param map The internal map
     * @param RUN_ON_APP Whether or not to show the screen to the driver station controller
     * @param pipeline The pipeline to use
     */
    protected OpenCVLoader(HardwareMap map, boolean RUN_ON_APP, OptimizedOpenCVPipeline pipeline){
        this.map = map;
        this.RUN_ON_APP = RUN_ON_APP;
        this.pipeline = pipeline;
        init();
    }

    /**
     * Initialize the pipeline and start streaming it
     */
    private void init() {

        if(RUN_ON_APP){
            int cameraMonitorViewId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());
            phoneCam = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName.class, RobotConfig.WEBCAM_NAME), cameraMonitorViewId);
        } else {
            phoneCam = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName.class, RobotConfig.WEBCAM_NAME));
        }


        phoneCam.setPipeline(pipeline);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }
}
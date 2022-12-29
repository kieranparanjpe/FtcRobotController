package org.firstinspires.ftc.teamcode.Global;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpenCVPipeline extends OpenCvPipeline {

    Telemetry telemetry;
    Mat mat = new Mat();

    OpenCvCamera camera;

    int position = 0;

    static final Rect Left = new Rect(
            new Point(0, 350),
            new Point(1920 / 3, 850));
    static final Rect Center = new Rect(
            new Point(1920 / 3, 350),
            new Point((1920 / 3) * 2, 850));
    static final Rect Right = new Rect(
            new Point((1920 / 3) * 2, 350),
            new Point(1920, 850));

    public OpenCVPipeline(Telemetry t, HardwareMap hardwareMap)
    {
        telemetry = t;

        int camViewID = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName()
                );
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), camViewID);
        camera.setPipeline(this);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int i) {

            }
        });

    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(125, 128, 30);
        Scalar highHSV = new Scalar(200, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(Left);
        Mat center = mat.submat(Center);
        Mat right = mat.submat(Right);

        double leftValue = Core.sumElems(left).val[0] / Left.area() / 255;
        double centerValue = Core.sumElems(center).val[0] / Center.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / Right.area() / 255;
/*
        telemetry.addData("Left Value: ", leftValue);
        telemetry.addData("Center Value: ", centerValue);
        telemetry.addData("Right Value: ", rightValue);
*/
        Rect range = new Rect();

        if (leftValue > rightValue && leftValue > centerValue) {
            position = 0;
            range = Left;
        }
        if (centerValue >= rightValue && centerValue >= leftValue) {
            position = 1;
            range = Center;
        }
        if (rightValue > leftValue && rightValue > centerValue)
        {
            position = 2;
            range = Right;
        }

        telemetry.addData("Height: ", position);

        //Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar highlight = new Scalar(255, 0 , 0);

        Imgproc.rectangle(mat, range, highlight);

        telemetry.update();

        return mat;
    }

    public int GetLocation()
    {
        return position;
    }

    public void Stop()
    {
        camera.closeCameraDeviceAsync(() -> {});
    }

}

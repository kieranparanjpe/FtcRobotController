package org.firstinspires.ftc.teamcode.Global;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Scalar;

public class ColorSensor {

    private RevColorSensorV3 revColorSensorV3 = null;
    private com.qualcomm.robotcore.hardware.ColorSensor colorSensor = null;

    private ColourRange[] ranges = new ColourRange[0];
    public double distance = 41;
    private Telemetry telemetry;

    public ColorSensor(RevColorSensorV3 c, ColourRange[] ranges, Telemetry t)
    {
        telemetry = t;
        revColorSensorV3 = c;
        this.ranges = ranges;
    }

    public ColorSensor(com.qualcomm.robotcore.hardware.ColorSensor c, ColourRange[] ranges, Telemetry t)
    {
        telemetry = t;
        colorSensor = c;
        this.ranges = ranges;

    }

    public ColorSensor(RevColorSensorV3 c, Telemetry t)
    {
        telemetry = t;
        revColorSensorV3 = c;
    }

    public ColorSensor(com.qualcomm.robotcore.hardware.ColorSensor c, Telemetry t)
    {
        telemetry = t;
        colorSensor = c;
    }

    public double Distance()
    {
        if(revColorSensorV3 != null)
            return revColorSensorV3.getDistance(DistanceUnit.MM);

        return Double.NaN;
    }

    public Scalar Color()
    {
        float[] hsv = new float[3];

        if(revColorSensorV3 != null)
        {
            Color.colorToHSV( revColorSensorV3.getNormalizedColors().toColor(), hsv);
            telemetry.addData("Color Sensor Rev v3 ", 255 * (hsv[0] / 360) + ", " + hsv[1] * 255 + ", " + hsv[2] * 255);
        }
        else
        {
            Color.colorToHSV(colorSensor.argb(), hsv);
            telemetry.addData("Color Sensor Normal ", 255 * (hsv[0] / 360) + ", " + hsv[1] * 255 + ", " + hsv[2] * 255);
        }

        hsv[0] = 255 * (hsv[0] / 360);
        hsv[1] = 255 * hsv[1];
        hsv[2] = 255 * hsv[2];


        return new Scalar(hsv[0], hsv[1], hsv[2]);
    }

    public boolean Detected()
    {
        float[] hsv = new float[3];

        if(revColorSensorV3 != null)
        {
            Color.colorToHSV( revColorSensorV3.getNormalizedColors().toColor(), hsv);
            telemetry.addData("Color Sensor Rev v3 ",255 * (hsv[0] / 360) + ", " + hsv[1] * 255 + ", " + hsv[2] * 255);
        }
        else
        {
            Color.colorToHSV(colorSensor.argb(), hsv);
            telemetry.addData("Color Sensor Rev v3 ", 255 * (hsv[0] / 360) + ", " + hsv[1] * 255 + ", " + hsv[2] * 255);
        }

        hsv[0] = 255 * (hsv[0] / 360);
        hsv[1] = 255 * hsv[1];
        hsv[2] = 255 * hsv[2];
        boolean detect = false;

        for(ColourRange r : ranges)
        {
            detect = (hsv[0] > r.low.val[0] && hsv[1] > r.low.val[1] && hsv[2] > r.low.val[2] && hsv[0] < r.high.val[0] && hsv[1] < r.high.val[1] && hsv[2] < r.high.val[2]);
            if(detect)
                return true;

        }

        return false;
    }

}

class ColourRange
{
    public Scalar low;
    public Scalar high;

    public ColourRange(Scalar low, Scalar high)
    {
        this.low = low;
        this.high = high;
    }
}

package org.firstinspires.ftc.teamcode.Global;

public class VectorData
{
    public double x = 0;
    public double y = 0;
    public double z = 0;

    public VectorData(double x, double y)
    {
        this.x = x;
        this.y = y;
    }

    public VectorData(double x, double y, double z)
    {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public void SetMag(double mag)
    {
        double direction = (Math.atan2(x, y));

        x = Math.sin(direction) * mag;
        y = Math.cos(direction) * mag;
    }

    public static VectorData SetMag(double mag, VectorData f)
    {
        VectorData from =  new VectorData(f.x, f.y);
        double direction = (Math.atan2(from.x, from.y));

        from.x = Math.sin(direction) * mag;
        from.y = Math.cos(direction) * mag;

        return from;
    }

    public static VectorData VectorFromDirection(double direction, double magnitude)
    {
        return new VectorData(Math.sin(Math.toRadians(direction)) * magnitude, Math.cos(Math.toRadians(direction)) * magnitude);
    }

    public static VectorData ComputePower(double direction, double magnitude)
    {
        VectorData v = VectorFromDirection(direction, 1);
        return SetMag(magnitude, new VectorData(v.x + v.y, -v.x + v.y));
    }

    public double Magnitude()
    {
        return Math.sqrt(x*x + y*y + z*z);
    }
}

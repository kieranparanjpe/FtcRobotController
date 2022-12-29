package org.firstinspires.ftc.teamcode.Global;

public class GyroPIDController extends PIDController {

    public GyroPIDController(double p, double i, double d)
    {
        super(p, i, d);
    }

    @Override
    public double Compute(double goal, double current)
    {
        this.current = current;
        if(this.goal != goal)
            lastGoal = this.goal;

        this.goal = goal;
        error = AngleWrap( goal - current);

        integral += error * timer.time();

        derivative = (error - last) / timer.time();

        last = error;
        timer.reset();

        return (error * kProportional) + (integral * kIntegral) + (derivative * kDerivative);
    }

    private double AngleWrap(double radians)
    {
        if(radians > Math.PI)
            radians -= Math.PI * 2;
        else if (radians < -Math.PI)
            radians += Math.PI * 2;

        return radians;
    }
}

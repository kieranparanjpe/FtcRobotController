package org.firstinspires.ftc.teamcode.Global;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    public double kProportional = 1, kIntegral = 1, kDerivative = 0;
    public double last = 0, integral = 0, derivative, error, current, lastGoal, goal;

    protected ElapsedTime timer = new ElapsedTime();
    protected ElapsedTime derivativeTimer = new ElapsedTime();
    protected double lastDerivativeTime = -10;

    public PIDController(double p, double i, double d)
    {
        kProportional = p;
        kIntegral = i;
        kDerivative = d;
    }

    public double Compute(double goal, double current)
    {
        this.current = current;
        if(this.goal != goal)
            lastGoal = this.goal;

        this.goal = goal;
        error = goal - current;

        integral += error * timer.time();

        derivative = (error - last) / timer.time();

        last = error;
        timer.reset();

        return (error * kProportional) + (integral * kIntegral) + (derivative * kDerivative);
    }

    public double Completed(double accuracy)
    {
        //return Math.abs(error) < 1;

        if(Math.abs(derivative) <= (1 - accuracy) / (kProportional * 10))
        {
            if(derivativeTimer.seconds() - lastDerivativeTime > 0.25 * accuracy)
                return 1;
        }
        else
        {
            lastDerivativeTime = derivativeTimer.seconds();
            return Math.abs((current - lastGoal) / (goal - lastGoal));
        }
        return Math.abs((current - lastGoal) / (goal - lastGoal));
    }

    public double Completed()
    {
        //return Math.abs(error) < 1;

        if(Math.abs(derivative) == 0)
        {
            if(derivativeTimer.seconds() - lastDerivativeTime > 0.25)
                return 1;
        }
        else
        {
            lastDerivativeTime = derivativeTimer.seconds();
            double p = Math.abs((current - lastGoal) / (goal - lastGoal));
            return p == 1 ? 0.99 : p;
        }
        return Math.abs((current - lastGoal) / (goal - lastGoal));
    }

}

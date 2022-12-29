package org.firstinspires.ftc.teamcode.Global;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoData {

    public Servo servo;
    public double startPosition;
    public double targetPosition;

    public double currentPosition = 0;

    public ServoData(String servo, double startPosition, double targetPosition, HardwareMap hardwareMap, Servo.Direction direction)
    {
        this.servo = hardwareMap.get(Servo.class, servo);
        this.startPosition = startPosition;
        this.targetPosition = targetPosition;

        this.servo.setDirection(direction);


        this.servo.setPosition(startPosition);


        currentPosition = startPosition;

    }

    public void SetPosition(boolean end)
    {
        if(end)
            currentPosition = targetPosition;
        else
            currentPosition = startPosition;

        servo.setPosition(currentPosition);

    }

    public void SetPosition(double position)
    {
        currentPosition = position;
        servo.setPosition(currentPosition);
    }

    public void ChangePosition(double increment)
    {
        currentPosition += increment;
        servo.setPosition(currentPosition);
    }
}

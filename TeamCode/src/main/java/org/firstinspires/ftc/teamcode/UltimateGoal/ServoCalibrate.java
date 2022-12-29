package org.firstinspires.ftc.teamcode.UltimateGoal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Global.ServoData;
@Disabled

@Autonomous(name="ServoCalibrate", group="Autonomous")
public class ServoCalibrate extends RobotFunctions
{
    private ElapsedTime runtime = new ElapsedTime();

    private ServoData servoData;

    @Override
    public void runOpMode() throws InterruptedException
    {

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        //In parenthesis -> name in quotes, start position, target position, don't change, direction (Servo.Direction.FORWARD or Servo.Direction.REVERSE)
        servoData = new ServoData("ServoNameHere", 0.0, 1.0, hardwareMap, Servo.Direction.FORWARD);

        SetServoPosition(servoData.servo, servoData.startPosition);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Servo Start Position", servoData.startPosition);
        telemetry.update();

        waitForStart();

        runtime.reset();

        SetServoPosition(servoData.servo, servoData.targetPosition);

        telemetry.addData("Runtime: ", runtime.time());
        telemetry.addData("Servo Target Position", servoData.targetPosition);
        telemetry.update();

        //Drive forward
    }
}
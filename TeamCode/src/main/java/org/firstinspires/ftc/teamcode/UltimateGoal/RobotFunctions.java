package org.firstinspires.ftc.teamcode.UltimateGoal;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Global.IMUData;
import org.firstinspires.ftc.teamcode.Global.VectorData;

/**
    This file contains the underlying functions to be used during the 2020/2021 season
**/


@Disabled
public abstract class RobotFunctions extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    //region Servo

    public void SetServoPosition(Servo servo, double position)
    {
        servo.setPosition(position);
    }

    //endregion

    //region Time

    public void TurnMotorTime(DcMotor motor, double power, long time)
    {
        motor.setPower(power);
        sleep(time);
        motor.setPower(0);
    }

    public void DriveAngleTime(DriveBaseData driveBaseData, double power, long time, double angle)
    {
        VectorData move = AngleToVectorData(angle);

        double left = Range.clip((move.x + move.y) * power, -1, 1);
        double right = Range.clip((move.x - move.y) * power, -1, 1);

        driveBaseData.SetPower(left, right, right, left);

        sleep(time);

        driveBaseData.SetPower(0);
    }

    public void DriveFrontBackTime(DriveBaseData driveBaseData, double power, long time)
    {
        driveBaseData.SetPower(power);

        sleep(time);

        driveBaseData.SetPower(0);
    }

    public void DriveLeftRightTime(DriveBaseData driveBaseData, double power, long time)
    {
        driveBaseData.SetPower(power, -power, -power, power);

        sleep(time);

        driveBaseData.SetPower(0);
    }

    //endregion

    //region Encoders

    public void TurnMotorDistance(DcMotor motor, double power, double rotations, double timeoutRedundancy, int encoderTicks)
    {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int encoderTicksPerRotation = encoderTicks;

        int newPosition = (int)(motor.getCurrentPosition() + Math.round(encoderTicksPerRotation * rotations));

        motor.setTargetPosition(newPosition);

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setPower(Math.abs(power));

        runtime.reset();

        while (opModeIsActive() && runtime.time() < timeoutRedundancy && motor.isBusy())
        {
            telemetry.addLine("Motor: Running");

            telemetry.update();
        }

        telemetry.addLine("Motor: Complete");

        telemetry.update();

        motor.setPower(0);

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(100);
    }

    public void DriveAngleDistance(DriveBaseData driveBaseData, double power, double distance, double angle, double timeoutRedundancy)
    {
        driveBaseData.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);

        VectorData move = AngleToVectorData(angle);

        double left = Range.clip((move.x + move.y), -1, 1);
        double right = Range.clip((move.x - move.y), -1, 1);

        telemetry.addData("Move", left + ", " + right);
        telemetry.update();

    

        int newPositionLeftFront = (int)(driveBaseData.leftFront.getCurrentPosition() + Math.round(distance * driveBaseData.ticksPerCentimeter * left));
        int newPositionRightFront = (int)(driveBaseData.rightFront.getCurrentPosition() + Math.round(distance * driveBaseData.ticksPerCentimeter * right));
        int newPositionLeftBack = (int)(driveBaseData.leftBack.getCurrentPosition() + Math.round(distance * driveBaseData.ticksPerCentimeter * right));
        int newPositionRightBack = (int)(driveBaseData.rightBack.getCurrentPosition() + Math.round(distance * driveBaseData.ticksPerCentimeter * left));

        driveBaseData.SetTargetPosition(newPositionLeftFront, newPositionRightFront, newPositionLeftBack, newPositionRightBack);

        driveBaseData.SetMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveBaseData.SetPower(Math.abs(power));

        runtime.reset();

        while (opModeIsActive() && runtime.time() < timeoutRedundancy && (driveBaseData.leftFront.isBusy() && driveBaseData.rightFront.isBusy() && driveBaseData.leftBack.isBusy() && driveBaseData.rightBack.isBusy()))
        {
            telemetry.addLine("Motors: Running");

            telemetry.addData("Move", left + ", " + right);

            telemetry.update();
        }
        telemetry.addData("Move", left + ", " + right);

        telemetry.addLine("Motors: Complete");

        telemetry.update();

        driveBaseData.SetPower(0);

        driveBaseData.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);


        sleep(100);
    }

    public void DriveFrontBackDistance(DriveBaseData driveBaseData, double power, double distance, double timeoutRedundancy) {
        driveBaseData.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newPositionLeftFront = (int) (driveBaseData.leftFront.getCurrentPosition() + Math.round(distance * driveBaseData.ticksPerCentimeter));
        int newPositionRightFront = (int) (driveBaseData.rightFront.getCurrentPosition() + Math.round(distance * driveBaseData.ticksPerCentimeter));
        int newPositionLeftBack = (int) (driveBaseData.leftBack.getCurrentPosition() + Math.round(distance * driveBaseData.ticksPerCentimeter));
        int newPositionRightBack = (int) (driveBaseData.rightBack.getCurrentPosition() + Math.round(distance * driveBaseData.ticksPerCentimeter));

        int pos = driveBaseData.leftFront.getCurrentPosition();

        driveBaseData.SetTargetPosition(newPositionLeftFront, newPositionRightFront, newPositionLeftBack, newPositionRightBack);

        driveBaseData.SetMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveBaseData.SetPower(Math.abs(power));

        runtime.reset();

        while (opModeIsActive() && runtime.time() < timeoutRedundancy && (driveBaseData.leftFront.isBusy() && driveBaseData.rightFront.isBusy() && driveBaseData.leftBack.isBusy() && driveBaseData.rightBack.isBusy())) {
            telemetry.addData("Motor", pos);
            telemetry.addLine("Motors: Running");

            telemetry.update();
        }

        telemetry.addLine("Motors: Complete");

        telemetry.update();

        driveBaseData.SetPower(0);

        driveBaseData.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);


        sleep(100);

    }

    public void DriveLeftRightDistance(DriveBaseData driveBaseData, double power, double distance, double timeoutRedundancy)
    {
        driveBaseData.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newPositionLeftFront = (int)(driveBaseData.leftFront.getCurrentPosition() + Math.round(distance * driveBaseData.ticksPerCentimeter));
        int newPositionRightFront = (int)(driveBaseData.rightFront.getCurrentPosition() + Math.round(-distance * driveBaseData.ticksPerCentimeter));
        int newPositionLeftBack = (int)(driveBaseData.leftBack.getCurrentPosition() + Math.round(-distance * driveBaseData.ticksPerCentimeter));
        int newPositionRightBack = (int)(driveBaseData.rightBack.getCurrentPosition() + Math.round(distance * driveBaseData.ticksPerCentimeter));

        driveBaseData.SetTargetPosition(newPositionLeftFront, newPositionRightFront, newPositionLeftBack, newPositionRightBack);

        driveBaseData.SetMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveBaseData.SetPower(Math.abs(power));

        runtime.reset();

        while (opModeIsActive() && runtime.time() < timeoutRedundancy && (driveBaseData.leftFront.isBusy() && driveBaseData.rightFront.isBusy() && driveBaseData.leftBack.isBusy() && driveBaseData.rightBack.isBusy()))
        {
            telemetry.addLine("Motors: Running");

            telemetry.update();
        }

        telemetry.addLine("Motors: Complete");

        telemetry.update();

        driveBaseData.SetPower(0);

        driveBaseData.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);


        sleep(100);

    }

    //endregion

    //region ColourSensor

    public void TurnMotorColour(DcMotor motor, double power, VectorData[] colourRange, NormalizedColorSensor colourSensor, double timeoutRedundancy)
    {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor.setPower(power);

        runtime.reset();

        VectorData colour = new VectorData(0, 0, 0);

        do
        {
            float[] hsv = new float[3];

            NormalizedRGBA colourSensorColour = colourSensor.getNormalizedColors();
            Color.colorToHSV(colourSensorColour.toColor(), hsv);
            colour = new VectorData(hsv[0], hsv[1], hsv[2]);

            telemetry.addLine("Motors: Running");
            telemetry.addLine("Hue: " + colour.x + " Saturation: " + colour.y + " Value: " + colour.z);

            telemetry.update();
        }
        while (opModeIsActive() && runtime.time() < timeoutRedundancy && CompareHSV(colour, colourRange));

        telemetry.addLine("Motor: Complete");

        telemetry.update();

        motor.setPower(0);

        sleep(100);
    }

    public void DriveAngleColour(DriveBaseData driveBaseData, double power, double angle, VectorData[] colourRange, NormalizedColorSensor colourSensor, double timeoutRedundancy)
    {
        driveBaseData.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);

        VectorData move = AngleToVectorData(angle);

        double left = Range.clip((move.x + move.y) * power, -1, 1);
        double right = Range.clip((move.x - move.y) * power, -1, 1);

        driveBaseData.SetPower(left, right, right, left);

        runtime.reset();

        VectorData colour = new VectorData(0, 0, 0);

        do
        {
            float[] hsv = new float[3];

            NormalizedRGBA colourSensorColour = colourSensor.getNormalizedColors();
            Color.colorToHSV(colourSensorColour.toColor(), hsv);
            colour = new VectorData(hsv[0], hsv[1], hsv[2]);

            telemetry.addLine("Motors: Running");
            telemetry.addLine("Hue: " + colour.x + " Saturation: " + colour.y + " Value: " + colour.z);

            telemetry.update();
        }
        while (opModeIsActive() && runtime.time() < timeoutRedundancy && CompareHSV(colour, colourRange));

        telemetry.addLine("Motors: Complete");

        telemetry.update();

        driveBaseData.SetPower(0);

        sleep(100);
    }

    public void DriveFrontBackColour(DriveBaseData driveBaseData, double power, VectorData[] colourRange, NormalizedColorSensor colourSensor, double timeoutRedundancy)
    {
        driveBaseData.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveBaseData.SetPower(power);

        runtime.reset();

        VectorData colour = new VectorData(0, 0, 0);

        do
        {
            float[] hsv = new float[3];

            NormalizedRGBA colourSensorColour = colourSensor.getNormalizedColors();
            Color.colorToHSV(colourSensorColour.toColor(), hsv);
            colour = new VectorData(hsv[0], hsv[1], hsv[2]);

            telemetry.addLine("Motors: Running");
            telemetry.addLine("Hue: " + colour.x + " Saturation: " + colour.y + " Value: " + colour.z);

            telemetry.update();
        }
        while (opModeIsActive() && runtime.time() < timeoutRedundancy && CompareHSV(colour, colourRange));

        telemetry.addLine("Motors: Complete");

        telemetry.update();

        driveBaseData.SetPower(0);

        sleep(100);
    }

    public void DriveLeftRightColour(DriveBaseData driveBaseData, double power, VectorData[] colourRange, NormalizedColorSensor colourSensor, double timeoutRedundancy)
    {
        driveBaseData.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveBaseData.SetPower(power, -power, -power, power);

        runtime.reset();

        VectorData colour = new VectorData(0, 0, 0);

        do
        {
            float[] hsv = new float[3];

            NormalizedRGBA colourSensorColour = colourSensor.getNormalizedColors();
            Color.colorToHSV(colourSensorColour.toColor(), hsv);
            colour = new VectorData(hsv[0], hsv[1], hsv[2]);

            telemetry.addLine("Motors: Running");
            telemetry.addLine("Hue: " + colour.x + " Saturation: " + colour.y + " Value: " + colour.z);

            telemetry.update();
        }
        while (opModeIsActive() && runtime.time() < timeoutRedundancy && CompareHSV(colour, colourRange));

        telemetry.addLine("Motors: Complete");

        telemetry.update();

        driveBaseData.SetPower(0);

        sleep(100);
    }

    //endregion

    //region Gyro


    //slow down as we come closer to target
    public void TurnGyro(DriveBaseData driveBaseData, double power, double angle, IMUData imuData, double timeoutRedundancy)
    {
        sleep(200);

        driveBaseData.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentAngle = imuData.HeadingAngle();
        angle = Range.clip(angle, -180, 180);

        if(angle > currentAngle)
        {
            driveBaseData.SetPower(-power, power, -power, power);
            runtime.reset();

            while (opModeIsActive() && runtime.time() < timeoutRedundancy && angle >= currentAngle + 10)
            {
                currentAngle = imuData.HeadingAngle();
                telemetry.addLine("Motors: Running");
                telemetry.addLine("Current Angle: " + currentAngle);

                telemetry.update();
            }
        }
        else if(angle < currentAngle)
        {
            driveBaseData.SetPower(power, -power, power, -power);
            runtime.reset();

            while (opModeIsActive() && runtime.time() < timeoutRedundancy && angle <= currentAngle - 10)
            {
                currentAngle = imuData.HeadingAngle();
                telemetry.addLine("Motors: Running");
                telemetry.addLine("Current Angle: " + currentAngle);

                telemetry.update();
            }
        }

        telemetry.addLine("Motors: Complete");

        telemetry.update();

        driveBaseData.SetPower(0);
        sleep(100);

        //return true;
    }

    //endregion

    //region Utility

    private VectorData AngleToVectorData(double angle)
    {
        VectorData move = new VectorData(0 , 0);

        angle = Math.toRadians(angle);
        move.y = Math.sin(angle);
        move.x = Math.cos(angle);
        return move;
    }

    private boolean CompareHSV(VectorData colour, VectorData[] range)
    {
        if(colour.x >= range[0].x && colour.x <= range[1].x && colour.y >= range[0].y && colour.y <= range[1].y && colour.z >= range[0].z && colour.z <= range[1].z)
            return true;

        return false;
    }

    protected double Lerp(double a, double b)
    {
        return ((b - a) / 2) + a;
    }

    protected boolean Range(double a, double b, double range)
    {
        if(a > b)
            return a - b < range;

        return b - a < range;
    }

    //endregion

}



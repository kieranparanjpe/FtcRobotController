package org.firstinspires.ftc.teamcode.UltimateGoal;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@Disabled

@TeleOp(name="RobotMovement", group="TeleOp")
public class RobotMovement extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftBack;
    private DcMotorEx leftFront;
    private DcMotorEx rightBack;
    private DcMotorEx rightFront;

    private DcMotorEx intakeMotor;
    private DcMotorEx shooterMotor;

    private NormalizedColorSensor colourSensor;

    @Override
    public void init()
    {
        leftBack = hardwareMap.get(DcMotorEx.class, "LeftBack");
        leftFront = hardwareMap.get(DcMotorEx.class,"LeftFront");
        rightBack = hardwareMap.get(DcMotorEx.class,"RightBack");
        rightFront = hardwareMap.get(DcMotorEx.class,"RightFront");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "ShooterMotor");

        colourSensor = hardwareMap.get(NormalizedColorSensor.class, "ColourSensor");

        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
    }

    @Override
    public void loop()
    {
        double leftY = gamepad1.left_stick_y; //driving
        double leftX = gamepad1.left_stick_x;
        double rightX  =  gamepad1.right_stick_x; //turning

        double leftBackPower = Range.clip(leftY - leftX + rightX, -1.0, 1.0); //-1+-1=-1
        double leftFrontPower = Range.clip(leftY + leftX + rightX, -1.0, 1.0); //(-1) - (-1) = 0
        double rightBackPower = Range.clip(leftY + leftX - rightX, -1.0, 1.0); // -1+-1=-1
        double rightFrontPower = Range.clip(leftY - leftX - rightX, -1.0, 1.0); //-1--1=0

      //  double leftBackPower = Range.clip(leftY - leftX - rightX, -1.0, 1.0); //-1+-1=-1
       // double rightBackPower = Range.clip(leftY + leftX + rightX, -1.0, 1.0); // -1+-1=-1
       // double leftFrontPower = Range.clip(leftY + leftX - rightX, -1.0, 1.0); //(-1) - (-1) = 0
       // double rightFrontPower = Range.clip(leftY - leftX + rightX, -1.0, 1.0); //-1--1=0

       // double leftBackPower = Range.clip(rightX, -1.0, 1.0); //-1+-1=-1
       // double rightBackPower = Range.clip(-rightX, -1.0, 1.0); // -1+-1=-1
       // double leftFrontPower = Range.clip(rightX, -1.0, 1.0); //(-1) - (-1) = 0
       // double rightFrontPower = Range.clip(-rightX, -1.0, 1.0); //-1--1=0



        leftBack.setPower(leftBackPower);
        leftFront.setPower(leftFrontPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);

        int shooterPower = Range.clip((int)gamepad1.right_trigger - (int)gamepad1.left_trigger, -1, 1);

        shooterMotor.setPower(shooterPower);

        int intakePower = 0;

        if(gamepad1.right_bumper)
            intakePower = 1;
        else if (gamepad1.left_bumper)
            intakePower = -1;

        intakeMotor.setPower(intakePower);


        telemetry.addLine(runtime.toString());

        float[] hsv = new float[3];

        NormalizedRGBA colourSensorColour = colourSensor.getNormalizedColors();
        Color.colorToHSV(colourSensorColour.toColor(), hsv);
        telemetry.addLine("Motors: Running");
        telemetry.addLine("Hue: " + hsv[0] + " Saturation: " + hsv[1] + " Value: " + hsv[2]);

        telemetry.update();

        telemetry.update();

    }
}
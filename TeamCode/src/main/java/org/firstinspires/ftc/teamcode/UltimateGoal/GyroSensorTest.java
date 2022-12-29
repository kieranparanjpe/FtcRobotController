package org.firstinspires.ftc.teamcode.UltimateGoal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled

@TeleOp(name = "GyroTest", group = "TeleOp")
public class GyroSensorTest extends LinearOpMode {

    BNO055IMU imu;
    Orientation angles;

    public void runOpMode()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        DriveBaseData db = new DriveBaseData("LeftFront","RightFront","LeftBack", "RightBack", 75, 1440, hardwareMap);

        waitForStart();

        while (opModeIsActive())
        {
            double p = gamepad1.left_stick_y;
            db.SetPower(p, -p, p, -p);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            telemetry.addData("Z", angles.firstAngle);
            telemetry.addData("Y", angles.secondAngle);
            telemetry.addData("X", angles.thirdAngle);
            telemetry.update();
        }
    }
}

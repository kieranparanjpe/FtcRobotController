package org.firstinspires.ftc.teamcode.UltimateGoal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Global.ServoData;

@Disabled

@Autonomous(name="LeagueOneAuto", group="Autonomous")
public class LeagueOneAuto extends RobotFunctions
{
    private ElapsedTime runtime = new ElapsedTime();

    private DriveBaseData driveBaseData = null;

    private ServoData wobbleServo;


    @Override
    public void runOpMode() throws InterruptedException
    {

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        driveBaseData = new DriveBaseData("LeftFront","RightFront","LeftBack", "RightBack", 75, 1440, hardwareMap);
        wobbleServo = new ServoData("WobbleServoArm", 0.0, 0.5, hardwareMap, Servo.Direction.FORWARD);

        driveBaseData.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SetServoPosition(wobbleServo.servo, wobbleServo.targetPosition);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset();


        sleep(2000);

        //Drive forward
        DriveFrontBackDistance(driveBaseData, 1, 1700, 10);
        
        driveBaseData.SetPower(1, -1, 1, -1);
        sleep(1000);
        driveBaseData.SetPower(0,0,0,0);
    }
}



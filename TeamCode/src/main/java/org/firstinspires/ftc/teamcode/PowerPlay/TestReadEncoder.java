package org.firstinspires.ftc.teamcode.PowerPlay;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Global.RobotPowerPlay;
import org.firstinspires.ftc.teamcode.Global.SlidePosition;
import org.firstinspires.ftc.teamcode.UltimateGoal.DriveControlState;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

@Config
@TeleOp(name="Read Encoder", group="TeleOp")
public class TestReadEncoder extends LinearOpMode
{
    //change to gamepad2 for 2 drivers
    private Gamepad attachmentController;

    private RobotPowerPlay robot = null;

    private DriveControlState driveState = DriveControlState.DRIVING;

    private boolean manualSlide = false;
    private final double manualSlideSpeed = 50;


    private double pincherTimer = 0.5, currentPincherTimer;
    private boolean pincherState;
    private double armTimer = 0.5, currentArmTimer;

    private double maxDriveSpeed = 1;
    private double normalDriveSpeed = 0.7;
    private double minDriveSpeed = 0.2;

    FtcDashboard dashboard;
    public static double directDriveSpeed = 0.5;
    public static int testInt;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //region init
        telemetry.addData("Status", "Initializing");
        telemetry.update();


       // dashboard = FtcDashboard.getInstance();

        attachmentController = gamepad2;

        DcMotor m = hardwareMap.get(DcMotor.class, "m");
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    //    robot = new RobotPowerPlay(hardwareMap, this);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //endregion

        waitForStart();
       // robot.GetTargetLocation();


        while(opModeIsActive()) {
telemetry.addData("position: ", m.getCurrentPosition());
telemetry.update();
        }
    }


}


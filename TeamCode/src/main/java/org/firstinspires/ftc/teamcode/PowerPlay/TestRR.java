package org.firstinspires.ftc.teamcode.PowerPlay;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Global.RobotPowerPlay;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Autonomous(name = "Test RR", group = "Autonomous")
public class TestRR  extends LinearOpMode {

    ElapsedTime time = new ElapsedTime();


    RobotPowerPlay robot;
    @Override
    public void runOpMode() throws InterruptedException {
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot = new RobotPowerPlay(hardwareMap, this);

        SampleMecanumDrive drive = robot.drive;

        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(50, 0, Math.toRadians(-20)))
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .lineToLinearHeading(new Pose2d(50, 24, Math.toRadians(-90)))
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .lineToLinearHeading(new Pose2d(50, 0, Math.toRadians(-20)))
                        .build();

        waitForStart();

        drive.followTrajectoryAsync(trajectory1);

        ElapsedTime t = new ElapsedTime();
time.reset();
        while(opModeIsActive() && drive.isBusy()) {
            telemetry.addData("Time", t.milliseconds());
drive.update();
telemetry.addData("cycle", time.milliseconds());
time.reset();
sleep(20);
            telemetry.update();
        }
    }
}

 package org.firstinspires.ftc.teamcode.PowerPlay;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Global.OdometryBot;
import org.firstinspires.ftc.teamcode.Global.RobotPowerPlay;
import org.firstinspires.ftc.teamcode.Global.SlidePosition;
import org.firstinspires.ftc.teamcode.Global.State;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.*;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter; // Import the FileWriter class
import java.io.IOException;

import java.util.ArrayList;

 @Autonomous(name = "Autonomous Power Play", group = "Autonomous")
 public class AutonomousProgramPowerPlay extends LinearOpMode {

     private OdometryBot robot = null;

     private ArrayList<State> states = new ArrayList<State>();
     private ArrayList<State> asyncStates = new ArrayList<State>();


     ElapsedTime timer;

     //in seconds
     int delay = 5;

     @Override
     public void runOpMode() throws InterruptedException
     {

         //region init
         telemetry.addData("Status", "Initializing");
         telemetry.update();

         robot = new OdometryBot(hardwareMap, this);


         timer = new ElapsedTime();

         String chosen = "Auto: default park";
         boolean cycle = true;
         int target = 2;

         //AutoNoCycle();
         ThirteenVolts();

         while(!isStarted())
         {
             if(gamepad1.b)
             {
                 cycle = true;
                     BlueAutonomousRight();

                 chosen = "Auto: Blue Right; Cycle: " + cycle;
             }

             if(gamepad1.x)
             {
                 cycle = true;
                     RedAutonomousLeft();


                 chosen = "Auto: Red Left; Cycle: " + cycle;
             }

             target = robot.GetTargetLocation();

             telemetry.addData("Program to run: ", chosen);
             telemetry.addData("Target: ", target);
             telemetry.update();
         }

         telemetry.clear();

         robot.StopWebcam();

         SlidePosition pos = SlidePosition.LOW;

         //choose index to insert the parking code

         //remove if statement when ready to write parking code
         cycle = true;
         if(!cycle) {
             if (target == 1)
                 states.add(3, robot.new DriveDistancePID(850, -90, 0.35, 5000, 1));
             if (target == 2) {
                 //add nothing
             }
             if (target == 3)
                 states.add(3, robot.new DriveDistancePID(850, 90, 0.35, 5000, 1));
         }
         else
         {
             if (target == 1) {
                 RobotLog.d("PARK 1");
                 states.add(robot.new AllenCoordDrive(-35000, -66000, 0, 1000, 2, 0.9, true, false));
                 states.add(robot.new Wait(delay));
                 states.add(robot.new Wait(delay));
                 states.add(robot.new Wait(delay));
                 states.add(robot.new Wait(delay));
             }
             if (target == 2) {
                 RobotLog.d("PARK 2");
                 states.add(robot.new AllenCoordDrive( 10000, -66000, 0, 1000, 2, 0.9, true, false));
                 states.add(robot.new Wait(delay));
                 states.add(robot.new Wait(delay));
                 states.add(robot.new Wait(delay));
                 states.add(robot.new Wait(delay));
             }
             if (target == 3) {
                 RobotLog.d("PARK 2");
                 states.add(robot.new AllenCoordDrive( 70000, -66000, 0, 1000, 2, 0.9, true, false));
                 states.add(robot.new Wait(delay));
                 states.add(robot.new Wait(delay));
                 states.add(robot.new Wait(delay));
                 states.add(robot.new Wait(delay));
             }
         }

         robot.runtime.reset();


         State currentState = states.get(0);
         int step = 0;

         timer.reset();

         while(opModeIsActive() && currentState != null)
         {
             if(currentState.runAsync)
             {
                 asyncStates.add(currentState);
                 step++;
                 currentState = states.get(step);
             }

             for (State s : asyncStates)
             {
                 double p = s.Run();
                 if(p >= 1)
                     asyncStates.remove(s);
             }

             double progress = currentState.Run();
             robot.onTick();

             telemetry.addData("Step ", step);
             telemetry.addData("Step progress ", currentState.getProgress());
             if(progress >= 1)
             {
                 step++;
                 if(step >= states.size())
                     currentState = null;
                 else
                     currentState = states.get(step);
             }

             robot.SlideToPosition(robot.slidePosition, 1);
             robot.ArmToPosition();



            // telemetry.addData("Distance driven ", distanceToHub);
            // robot.outerColorSensor.Color();



             telemetry.addData("Target Zone", target);

            telemetry.addData("Cycle Time", timer.milliseconds());
            timer.reset();
             // telemetry.addLine("Progress: " + progress);
             telemetry.update();
         }

             // System.out.println("Successfully wrote to the file.");


     }
     private void AutoNoCycle()
     {
         states = new ArrayList<State>();

         states.add(robot.new Wait(delay));
         //first drive
         states.add(robot.new DriveDistancePID(50, 0, 0.5, 5000, 1));
         states.add(robot.new TurnGyroPID(0.2, 0, 1000, 1, false));
         //insert drive left / right
         states.add(robot.new DriveDistancePID(900, 0, 0.35, 5000, 1));
         states.add(robot.new TurnGyroPID(0.2, 0, 1000, 1, false));
     }

     private void TestRR()
     {
         /*
             TrajectorySequence trajectory1 = robot.drive.trajectorySequenceBuilder(new Pose2d())
                     .lineToLinearHeading(new Pose2d(15, 0, Math.toRadians(90)))
                     .lineToLinearHeading(new Pose2d(15, 15, Math.toRadians(180)))
                     .lineToLinearHeading(new Pose2d(0, 15, Math.toRadians(270)))
                     .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                     .build();

         states = new ArrayList<State>();

         states.add(robot.new Wait(delay));
         //first drive
         states.add(robot.new RoadRunnerFollowTrajectorySequence(trajectory1, 150000, false));

         states.add(robot.new Wait(delay));
         states.add(robot.new Wait(delay));
         states.add(robot.new Wait(delay));
         states.add(robot.new Wait(delay));*/

//         Trajectory trajectory1 = robot.drive.trajectoryBuilder(new Pose2d())
//                 .lineToLinearHeading(new Pose2d(40, 0, Math.toRadians(-19)))
//                 .build();
//
//         TrajectorySequence trajectory2 = robot.drive.trajectorySequenceBuilder(trajectory1.end())
//                 .lineToConstantHeading(new Vector2d(49, 0))
//                 .lineToLinearHeading(new Pose2d(49, 20, Math.toRadians(-90)))
//                 .back(3)
//                 .build();

     /*    Trajectory trajectory3 = robot.drive.trajectoryBuilder(trajectory2.end())
                 .lineToLinearHeading(new Pose2d(47.5, 20, Math.toRadians(-90)))
                 .build();*/

        /* Trajectory trajectory3 = robot.drive.trajectoryBuilder(trajectory2.end().plus(new Pose2d(0, 0, Math.toRadians(71))))
                 .lineToConstantHeading(new Vector2d(48, 20)
//                         SampleMecanumDrive.getVelocityConstraint(2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                         SampleMecanumDrive.getAccelerationConstraint(2))
                 )
                 .build();*/


         states = new ArrayList<State>();

         states.add(robot.new Wait(delay));
         //first drive
      //  states.add(robot.new SetSlidePosition(SlidePosition.HIGH));

         //states.add(robot.new RoadRunnerFollowTrajectory(trajectory1, 5000, false));
   //      states.add(robot.new SetSlidePosition(SlidePosition.DOWN));
        /* states.add(robot.new SetSlidePosition(SlidePosition.HIGH));
         states.add(robot.new ArmToPosition(true));
         states.add(robot.new Wait(4));

         states.add(robot.new SetSlidePosition(SlidePosition.HIGH));
         states.add(robot.new Wait(1));
         states.add(robot.new SetClaw(true));
         states.add(robot.new SetSlidePosition(SlidePosition.DOWN4));
         states.add(robot.new Wait(0.5));*/
         //states.add(robot.new RoadRunnerFollowTrajectorySequence(trajectory2, 10000, false));
        // states.add(robot.new RoadRunnerTurn(-71, 5000, false));
         //states.add(robot.new RoadRunnerFollowTrajectory(trajectory3, 5000, false));




         states.add(robot.new Wait(delay));
         states.add(robot.new Wait(delay));
         states.add(robot.new Wait(delay));
         states.add(robot.new Wait(delay));

         /*states.add(robot.new RoadRunnerFollowTrajectory(trajectory2, 5000, false));
         states.add(robot.new RoadRunnerFollowTrajectory(trajectory3, 5000, false));
         states.add(robot.new RoadRunnerFollowTrajectory(trajectory2, 5000, false));
         states.add(robot.new RoadRunnerFollowTrajectory(trajectory3, 5000, false));
         states.add(robot.new RoadRunnerFollowTrajectory(trajectory2, 5000, false));
         states.add(robot.new RoadRunnerFollowTrajectory(trajectory3, 5000, false));
         states.add(robot.new RoadRunnerFollowTrajectory(trajectory2, 5000, false));
         states.add(robot.new RoadRunnerFollowTrajectory(trajectory3, 5000, false));*/


     }

     private void ThirteenVolts()
     {
         states = new ArrayList<State>();

         //first drive
         //  states.add(robot.new SetSlidePosition(SlidePosition.HIGH));


         states.add(robot.new AllenCoordDrive(-30000, -60000, 0, 40000, 2, 1, false, false));
         states.add(robot.new Wait(0.5));
         //states.add(robot.new AllenCoordDrive(-5000, -52000, 0, 500, 2, 0.7, true, false));
         //states.add(robot.new Wait(2));

         states.add(robot.new AllenCoordDrive(-5000, -53000, 25, 500, 2, 1, true, false));
         //states.add(robot.new Wait(0.5));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGH));
         states.add(robot.new ArmToPosition(true));
         states.add(robot.new Wait(1.6));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGH));
         states.add(robot.new Wait(0.3));
         states.add(robot.new SetClaw(true));
         states.add(robot.new SetSlidePosition(SlidePosition.DOWN4));
         states.add(robot.new Wait(0.5));

         //states.add(robot.new Wait(delay));
         states.add(robot.new AllenCoordDrive(-10000, -66000, 90, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(1));
         states.add(robot.new AllenCoordDrive(-26000, -66000, 90, 1000, 2, 0.7, true, false));
         states.add(robot.new Wait(1));
         states.add(robot.new SetClaw(false));
         states.add(robot.new Wait(0.2));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGH));
         states.add(robot.new ArmToPosition(true));
         states.add(robot.new Wait(0.3));
         states.add(robot.new AllenCoordDrive(-18000, -66000, 60, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(1.2));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGH));
         states.add(robot.new Wait(0.3));
         states.add(robot.new SetClaw(true));
         states.add(robot.new SetSlidePosition(SlidePosition.DOWN3));
         states.add(robot.new Wait(0.5));
         states.add(robot.new AllenCoordDrive(-28000, -66000, 90, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(1.3));
         states.add(robot.new SetClaw(false));
         states.add(robot.new Wait(0.2));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGH));
         states.add(robot.new ArmToPosition(true));
         states.add(robot.new Wait(0.3));
         states.add(robot.new AllenCoordDrive(-18000, -66000, 60, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(1.2));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGH));
         states.add(robot.new Wait(0.3));
         states.add(robot.new SetClaw(true));
         states.add(robot.new SetSlidePosition(SlidePosition.DOWN2));
         states.add(robot.new Wait(0.5));
         states.add(robot.new AllenCoordDrive(-28000, -66000, 90, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(1.4));
         states.add(robot.new SetClaw(false));
         states.add(robot.new Wait(0.2));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGH));
         states.add(robot.new ArmToPosition(true));
         states.add(robot.new Wait(0.3));
         states.add(robot.new AllenCoordDrive(-19000, -66000, 60, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(1.2));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGH));
         states.add(robot.new Wait(0.3));
         states.add(robot.new SetClaw(true));
         states.add(robot.new SetSlidePosition(SlidePosition.DOWN1));
         states.add(robot.new Wait(0.7));
         states.add(robot.new AllenCoordDrive(-29000, -66000, 90, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(1.5));
         states.add(robot.new SetClaw(false));
         states.add(robot.new Wait(0.3));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGH));
         states.add(robot.new ArmToPosition(true));
         states.add(robot.new Wait(0.1));
         states.add(robot.new AllenCoordDrive(-19000, -66000, 60, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(1.2));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGH));
         states.add(robot.new Wait(0.5));
         states.add(robot.new SetClaw(true));
         states.add(robot.new SetSlidePosition(SlidePosition.DOWN));
         states.add(robot.new Wait(0.7));
         states.add(robot.new AllenCoordDrive(-29000, -66000, 90, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(1.5));
         states.add(robot.new SetClaw(false));
         states.add(robot.new Wait(0.5));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGH));
         states.add(robot.new ArmToPosition(true));
         states.add(robot.new Wait(0.1));
         states.add(robot.new AllenCoordDrive(-19000, -66000, 60, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(1.2));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGH));
         states.add(robot.new Wait(0.5));
         states.add(robot.new SetClaw(true));
         states.add(robot.new SetSlidePosition(SlidePosition.DOWN));
         states.add(robot.new ArmToPosition(true));
         states.add(robot.new Wait(0.5));
         //states.add(robot.new SetSlidePosition(SlidePosition.DOWN));


         // states.add(robot.new RoadRunnerTurn(-71, 5000, false));
         //states.add(robot.new RoadRunnerFollowTrajectory(trajectory3, 5000, false));




//         states.add(robot.new Wait(delay));
//         states.add(robot.new Wait(delay));
//         states.add(robot.new Wait(delay));
//         states.add(robot.new Wait(delay));

         /*states.add(robot.new RoadRunnerFollowTrajectory(trajectory2, 5000, false));
         states.add(robot.new RoadRunnerFollowTrajectory(trajectory3, 5000, false));
         states.add(robot.new RoadRunnerFollowTrajectory(trajectory2, 5000, false));
         states.add(robot.new RoadRunnerFollowTrajectory(trajectory3, 5000, false));
         states.add(robot.new RoadRunnerFollowTrajectory(trajectory2, 5000, false));
         states.add(robot.new RoadRunnerFollowTrajectory(trajectory3, 5000, false));
         states.add(robot.new RoadRunnerFollowTrajectory(trajectory2, 5000, false));
         states.add(robot.new RoadRunnerFollowTrajectory(trajectory3, 5000, false));*/


     }


     private void BlueAutonomousRight()
     {
         states = new ArrayList<State>();
         //first drive
         states.add(robot.new DriveDistancePID(900, 0, 0.4, 5000, 1));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGH));
         states.add(robot.new ArmToPosition(true));
         states.add(robot.new TurnGyroPID(1, -18, 2, 1, false));
         states.add(robot.new DriveDistancePID(25, 0, 0.3, 5000, 1));
         states.add(robot.new TurnColor(robot.slideColourSensor, -1, 0.05, 1, false));
         states.add(robot.new DriveDistancePID(30, 0, 0.5, 5000, 1));

         states.add(robot.new SetSlidePosition(SlidePosition.HIGH));
         states.add(robot.new Wait(20));
         states.add(robot.new SetClaw(true));
         states.add(robot.new Wait(0.5));
         states.add(robot.new DriveDistancePID(100, 180, 0.4, 5000, 1));
         states.add(robot.new SetSlidePosition(SlidePosition.GROUND));
         states.add(robot.new TurnGyroPID(0.5, 0, 2, 1, false));


     }

     private void RedAutonomousLeft()
     {
         states = new ArrayList<State>();


         //first drive
         states.add(robot.new DriveDistancePID(900, 0, 0.4, 5000, 1));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGH));
         states.add(robot.new ArmToPosition(true));
         states.add(robot.new TurnGyroPID(1, 18, 2, 1, false));
         states.add(robot.new DriveDistancePID(25, 0, 0.3, 5000, 1));
         states.add(robot.new TurnColor(robot.slideColourSensor, 1, 0.05, 1, false));
         states.add(robot.new DriveDistancePID(30, 0, 0.5, 5000, 1));

         states.add(robot.new SetSlidePosition(SlidePosition.HIGH));
         states.add(robot.new Wait(0.4));
         states.add(robot.new SetClaw(true));
         states.add(robot.new Wait(0.5));
         states.add(robot.new DriveDistancePID(100, 180, 0.4, 5000, 1));
         states.add(robot.new SetSlidePosition(SlidePosition.GROUND));
         states.add(robot.new Wait(0.3));
         states.add(robot.new TurnGyroPID(0.5, 0, 2, 1, false));
         states.add(robot.new DriveDistancePID(330, 0, 0.5, 5000, 1));
         states.add(robot.new TurnGyroPID(0.4, 90, 5, 1, false));
         states.add(robot.new Wait(1));
         states.add(robot.new SetClaw(true));
         states.add(robot.new SetSlidePosition(SlidePosition.DOWN1));
         states.add(robot.new SetClaw(true));

         states.add(robot.new DriveDistancePID(470, 180, 0.5, 5000, 1));
         states.add(robot.new DriveDistancePID(90, 0, 0.5, 5000, 1));
         states.add(robot.new SetSlidePosition(SlidePosition.DOWN4));
         states.add(robot.new DriveDistancePID(90, 180, 0.5, 5000, 1));
         states.add(robot.new SetClaw(false));
         states.add(robot.new Wait(0.6));

         states.add(robot.new SetSlidePosition(SlidePosition.LOW));
         states.add(robot.new TurnGyroPID(1, 75, 2, 1, false));
         states.add(robot.new DriveDistancePID(128, 0, 0.5, 5000, 1));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGH));
         states.add(robot.new ArmToPosition(true));
         states.add(robot.new Wait(1));
         states.add(robot.new TurnColor(robot.slideColourSensor, -1, 0.05, 1, false));
         states.add(robot.new DriveDistancePID(30, 0, 0.5, 5000, 1));

         states.add(robot.new SetSlidePosition(SlidePosition.HIGH));
         states.add(robot.new Wait(0.5));
         states.add(robot.new SetClaw(true));
         states.add(robot.new Wait(0.3));
         states.add(robot.new DriveDistancePID(80, 180, 0.3, 5000, 1));
         states.add(robot.new SetSlidePosition(SlidePosition.GROUND));
         states.add(robot.new ArmToPosition(true));
         states.add(robot.new Wait(0.3));
         states.add(robot.new DriveDistancePID(25, 180, 0.3, 5000, 1));

         states.add(robot.new TurnGyroPID(0.3, 0, 4, 1, false));




         //drive right / left depending on side
       //  states.add(robot.new DriveDistancePID(670, 270, 0.3, 5000, 1));

      //   states.add(robot.new DriveDistancePID(550, 0, 0.3, 5000, 1));

         //slide height is here

     }


 }

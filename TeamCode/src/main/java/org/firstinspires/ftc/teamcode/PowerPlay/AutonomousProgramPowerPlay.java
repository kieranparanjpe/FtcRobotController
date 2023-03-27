 package org.firstinspires.ftc.teamcode.PowerPlay;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Global.RobotPowerPlay;
import org.firstinspires.ftc.teamcode.Global.SlidePosition;
import org.firstinspires.ftc.teamcode.Global.State;

import java.util.*;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter; // Import the FileWriter class
import java.io.IOException;

import java.util.ArrayList;

 @Autonomous(name = "Autonomous Power Play", group = "Autonomous")
 public class AutonomousProgramPowerPlay extends LinearOpMode {

     private RobotPowerPlay robot = null;

     private ArrayList<State> states = new ArrayList<State>();
     private ArrayList<State> asyncStates = new ArrayList<State>();


     //in seconds
     int delay = 0;

     @Override
     public void runOpMode() throws InterruptedException
     {

         //region init
         telemetry.addData("Status", "Initializing");
         telemetry.update();

         robot = new RobotPowerPlay(hardwareMap, this);


         String chosen = "Auto: default park";
         boolean cycle = false;
         int target = 2;

         //AutoNoCycle();
         TestRR();

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
             if (target == 1)
                 states.add(robot.new DriveDistancePID(150, -90, 0.35, 5000, 1));
             if (target == 2) {
                 states.add(robot.new DriveDistancePID(410, 90, 0.35, 5000, 1));
             }
             if (target == 3) {
                 states.add(robot.new TurnGyroPID(0.3, 90, 4, 1, false));

                 states.add(robot.new DriveDistancePID(1575, 0, 0.35, 5000, 1));
             }
         }

         robot.runtime.reset();


         State currentState = states.get(0);
         int step = 0;

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
             robot.drive.update();

            // telemetry.addData("Distance driven ", distanceToHub);
            // robot.outerColorSensor.Color();



             telemetry.addData("Target Zone", target);


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
         Trajectory trajectory1 = robot.drive.trajectoryBuilder(new Pose2d())
                 .lineToLinearHeading(new Pose2d(50, 0, Math.toRadians(-20)))
                 .build();

         Trajectory trajectory2 = robot.drive.trajectoryBuilder(trajectory1.end())
                 .lineToLinearHeading(new Pose2d(50, 24, Math.toRadians(-90)))
                 .build();

         Trajectory trajectory3 = robot.drive.trajectoryBuilder(trajectory2.end())
                 .lineToLinearHeading(new Pose2d(50, 0, Math.toRadians(-20)))
                 .build();

         states = new ArrayList<State>();

         states.add(robot.new Wait(delay));
         //first drive
         states.add(robot.new RoadRunnerFollowTrajectory(trajectory1, 5000, false));
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

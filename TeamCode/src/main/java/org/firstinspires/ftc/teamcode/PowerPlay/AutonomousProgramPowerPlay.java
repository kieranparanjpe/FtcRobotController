 package org.firstinspires.ftc.teamcode.PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Global.RobotPowerPlay;
import org.firstinspires.ftc.teamcode.Global.SlidePosition;
import org.firstinspires.ftc.teamcode.Global.State;

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


         String chosen = "";
         boolean cycle = true;
         int target = 2;

         AutoNoCycle();

         while(!isStarted())
         {
             if(gamepad1.right_trigger > 0)
                 cycle = true;
             if(gamepad1.left_trigger > 0)
                 cycle = false;

             if(gamepad1.a)
             {
                 if(cycle)
                     BlueAutonomousLeft();
                 else
                     AutoNoCycle();

                 chosen = "Blue Left; Cycle: " + cycle;
             }

             if(gamepad1.b)
             {
                 if(cycle)
                     BlueAutonomousRight();
                 else
                     AutoNoCycle();

                 chosen = "Blue Right; Cycle: " + cycle;
             }

             if(gamepad1.x)
             {
                 if(cycle)
                     RedAutonomousLeft();
                 else
                     AutoNoCycle();

                 chosen = "Red Left; Cycle: " + cycle;
             }

             if(gamepad1.y)
             {
                 if(cycle)
                     RedAutonomousRight();
                 else
                     AutoNoCycle();

                 chosen = "Red Right; Cycle: " + cycle;
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
         int index = cycle ? 20 : 3;

         //remove if statement when ready to write parking code
         if(index == 3) {
             if (target == 1)
                 states.add(index, robot.new DriveDistancePID(850, -90, 0.35, 5000, 1));
             if (target == 2) {
                 //add nothing
             }
             if (target == 3)
                 states.add(index, robot.new DriveDistancePID(850, 90, 0.35, 5000, 1));
         }
         else
         {
             if (target == 1)
                 states.add(robot.new DriveDistancePID(80, -90, 0.35, 5000, 1));
             if (target == 2) {
                 states.add(robot.new DriveDistancePID(300, 90, 0.35, 5000, 1));
             }
             if (target == 3)
                 states.add(robot.new DriveDistancePID(900, 90, 0.35, 5000, 1));
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

            // telemetry.addData("Distance driven ", distanceToHub);
            // robot.outerColorSensor.Color();



             telemetry.addData("Target Zone", target);


             // telemetry.addLine("Progress: " + progress);
             telemetry.update();
         }
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
     private void BlueAutonomousLeft()
     {
         states = new ArrayList<State>();

         //first drive
         states.add(robot.new DriveDistancePID(90, 0, 0.3, 5000, 1));

         //drive right / left depending on side
         states.add(robot.new DriveDistancePID(670, 90, 0.3, 5000, 1));

         states.add(robot.new DriveDistancePID(550, 0, 0.3, 5000, 1));

         //slide height is here
         states.add(robot.new Wait(1f));
         states.add(robot.new Wait(1.5f));
         states.add(robot.new Wait(0.75));
         states.add(robot.new SetSlidePosition(SlidePosition.DOWN));
         states.add(robot.new DriveDistancePID(150, 180, 0.75, 5, 1));

         //go to carousel
         states.add(robot.new TurnGyroPID(1, 270, 2, 1, false));
         states.add(robot.new DriveDistancePID( 800, 180, 0.8, 5, 1));
         states.add(robot.new DriveDistanceColor( robot.slideColourSensor, 180, 0.2, 1450, false));
         states.add(robot.new DriveDistancePID( 175, 180, 0.2, 5, 1));
         states.add(robot.new DriveDistanceColor(robot.slideColourSensor, 270, 0.2, 130, false));
         states.add(robot.new DriveDistancePID(270, 270, 0.5, 2, 1));
         states.add(robot.new DriveDistancePID(30, 270, 0.1, 2, 1));
         states.add(robot.new Wait(3));
         states.add(robot.new DriveDistancePID(310, 90, 0.5, 5, 1));

         //park
         states.add(robot.new TurnGyroPID(1, 270, 2, 1, false));
         states.add(robot.new DriveDistancePID(2200, 0, 1, 8, 1));
         //states.add(robot.new TurnGyroPID(1, 270, 2, 1, false));
         states.add(robot.new DriveDistancePID(650, 0, 1, 8, 1));

     }

     private void BlueAutonomousRight()
     {
         states = new ArrayList<State>();

         //first drive
         states.add(robot.new DriveDistancePID(90, 0, 0.3, 5000, 1));

         //drive right / left depending on side
         states.add(robot.new DriveDistancePID(670, 270, 0.3, 5000, 1));

         states.add(robot.new DriveDistancePID(550, 0, 0.3, 5000, 1));

         //slide height is here
         states.add(robot.new Wait(1f));
         states.add(robot.new Wait(1.5f));
         states.add(robot.new Wait(0.75));
         states.add(robot.new SetSlidePosition(SlidePosition.DOWN));
         states.add(robot.new DriveDistancePID(150, 180, 0.75, 5, 1));

         //go to carousel
         states.add(robot.new TurnGyroPID(1, 270, 2, 1, false));
         states.add(robot.new DriveDistancePID( 800, 180, 0.8, 5, 1));
         states.add(robot.new DriveDistanceColor( robot.slideColourSensor, 180, 0.2, 1450, false));
         states.add(robot.new DriveDistancePID( 175, 180, 0.2, 5, 1));
         states.add(robot.new DriveDistanceColor(robot.slideColourSensor, 270, 0.2, 130, false));
         states.add(robot.new DriveDistancePID(270, 270, 0.5, 2, 1));
         states.add(robot.new DriveDistancePID(30, 270, 0.1, 2, 1));
         states.add(robot.new Wait(3));
         states.add(robot.new DriveDistancePID(310, 90, 0.5, 5, 1));

         //park
         states.add(robot.new TurnGyroPID(1, 270, 2, 1, false));
         states.add(robot.new DriveDistancePID(2200, 0, 1, 8, 1));
         //states.add(robot.new TurnGyroPID(1, 270, 2, 1, false));
         states.add(robot.new DriveDistancePID(650, 0, 1, 8, 1));

     }

     private void RedAutonomousLeft()
     {
         states = new ArrayList<State>();
         //first drive
         states.add(robot.new DriveDistancePID(950, 0, 0.3, 5000, 1));
         //states.add(robot.new TurnGyroPID(1, 25, 2, 1, false));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGH));
         states.add(robot.new ArmToPosition(true));
         states.add(robot.new Wait(2));
         states.add(robot.new TurnColor(robot.slideColourSensor, 1, 0.05, 15, false));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGH));
         states.add(robot.new Wait(1));

         states.add(robot.new SetClaw(true));

         states.add(robot.new SetSlidePosition(SlidePosition.DOWN1));
         states.add(robot.new Wait(2));
         states.add(robot.new TurnGyroPID(0.5, 0, 2, 1, false));
         states.add(robot.new DriveDistancePID(300, 0, 0.5, 5000, 1));
         states.add(robot.new TurnGyroPID(0.1, 90, 5, 1, false));
         states.add(robot.new DriveDistancePID(500, 180, 0.5, 5000, 1));
         states.add(robot.new DriveDistancePID(40, 0, 0.5, 5000, 1));
         states.add(robot.new SetSlidePosition(SlidePosition.DOWN4));
         states.add(robot.new DriveDistancePID(50, 180, 0.5, 5000, 1));
         states.add(robot.new SetClaw(false));
         states.add(robot.new Wait(1));

         states.add(robot.new SetSlidePosition(SlidePosition.HIGH));
         states.add(robot.new TurnGyroPID(1, 68, 2, 1, false));
         states.add(robot.new DriveDistancePID(95, 0, 0.5, 5000, 1));

         states.add(robot.new Wait(2));
         states.add(robot.new SetClaw(true));
         states.add(robot.new Wait(1));
         states.add(robot.new SetSlidePosition(SlidePosition.DOWN4));
         states.add(robot.new Wait(2));
         states.add(robot.new TurnGyroPID(0.5, 0, 2, 1, false));




         //drive right / left depending on side
       //  states.add(robot.new DriveDistancePID(670, 270, 0.3, 5000, 1));

      //   states.add(robot.new DriveDistancePID(550, 0, 0.3, 5000, 1));

         //slide height is here

     }

     private void RedAutonomousRight()
     {
         states = new ArrayList<State>();

         states.add(robot.new Wait(delay));

         //first drive
         states.add(robot.new DriveDistancePID(90, 0, 0.3, 5000, 1));

         //drive right / left depending on side
         states.add(robot.new DriveDistancePID(670, 270, 0.3, 5000, 1));

         states.add(robot.new DriveDistancePID(550, 0, 0.3, 5000, 1));

         //slide height is here
         states.add(robot.new Wait(1f));
         states.add(robot.new Wait(1.5f));
         states.add(robot.new Wait(0.75));
         states.add(robot.new SetSlidePosition(SlidePosition.DOWN));
         states.add(robot.new DriveDistancePID(75, 180, 0.75, 5, 1));

         //park
         states.add(robot.new TurnGyroPID(1, 90, 2, 1, false));
         states.add(robot.new DriveDistancePID(1500, 0, 1, 8, 1));


     }


 }

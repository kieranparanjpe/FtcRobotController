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
         boolean cycle = false;
         int target = 2;

         while(!isStarted())
         {
             if(gamepad1.right_trigger > 0)
                 cycle = true;
             if(gamepad1.left_trigger > 0)
                 cycle = false;

            /* if(gamepad1.a)
             {
                 if(cycle)
                     BlueAutonomous1All();
                 else
                     BlueAutonomous1NoDuck(delay);

                 chosen = "Blue 1; Cycle: " + cycle;
             }

             if(gamepad1.b)
             {
                 if(cycle)
                     BlueAutonomous2All();
                 else
                     BlueAutonomous2NoCycle(delay);

                 chosen = "Blue 2; Cycle: " + cycle;
             }

             if(gamepad1.x)
             {
                 if(cycle)
                     RedAutonomous1All();
                 else
                     RedAutonomous1NoDuck(delay);

                 chosen = "Red 1; Cycle: " + cycle;
             }

             if(gamepad1.y)
             {
                 if(cycle)
                     RedAutonomous2All();
                 else
                     RedAutonomous2NoDuck(delay);

                 chosen = "Red 2; Cycle: " + cycle;
             }
*/

             BlueAutonomous2NoCycle(delay);
             target = robot.GetTargetLocation();

             telemetry.addData("Program to run: ", chosen);
             telemetry.addData("Target: ", target);
             telemetry.update();
         }

         robot.StopWebcam();

         SlidePosition pos = SlidePosition.LOW;

         if(target == 1)
             states.add(3, robot.new DriveDistancePID(850, -90, 0.35, 5000, 1));
         if(target == 2) {
             //add nothing
         }
         if(target == 3)
             states.add(3, robot.new DriveDistancePID(850, 90, 0.35, 5000, 1));


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

            // telemetry.addData("Distance driven ", distanceToHub);
            // robot.outerColorSensor.Color();



             telemetry.addData("Target Zone", target);


             // telemetry.addLine("Progress: " + progress);
             telemetry.update();
         }



     }

     private void BlueAutonomous2All()
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
         states.add(robot.new DriveDistanceColor( robot.outerColorSensor, 180, 0.2, 1450, false));
         states.add(robot.new DriveDistancePID( 175, 180, 0.2, 5, 1));
         states.add(robot.new DriveDistanceColor(robot.outerColorSensor, 270, 0.2, 130, false));
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

     private void BlueAutonomous2NoCycle(int delay)
     {
         states = new ArrayList<State>();

         states.add(robot.new Wait(delay));
         //first drive
         states.add(robot.new DriveDistancePID(50, 0, 0.5, 5000, 1));
         states.add(robot.new TurnGyroPID(0.2, 0, 1000, 1, false));
         states.add(robot.new DriveDistancePID(900, 0, 0.35, 5000, 1));
         states.add(robot.new TurnGyroPID(0.2, 0, 1000, 1, false));

     }

     private void BlueAutonomous1All()
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
         states.add(robot.new DriveDistanceColor( robot.outerColorSensor, 180, 0.2, 1450, false));
         states.add(robot.new DriveDistancePID( 175, 180, 0.2, 5, 1));
         states.add(robot.new DriveDistanceColor(robot.outerColorSensor, 270, 0.2, 130, false));
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

     private void BlueAutonomous1NoDuck(int delay)
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
         states.add(robot.new DriveDistancePID(150, 180, 0.75, 5, 1));

         //park
         states.add(robot.new TurnGyroPID(1, 270, 2, 1, false));
         states.add(robot.new DriveDistancePID(1500, 0, 1, 8, 1));

     }

     private void RedAutonomous2All()
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
         states.add(robot.new DriveDistancePID( 800, 270, 0.8, 5, 1));
         states.add(robot.new DriveDistanceColor( robot.outerColorSensor, 270, 0.2, 1450, false));
         states.add(robot.new DriveDistancePID( 175, 270, 0.2, 5, 1));
         states.add(robot.new DriveDistanceColor(robot.outerColorSensor, 180, 0.2, 130, false));
         states.add(robot.new DriveDistancePID(30, 180, 0.5, 2, 1));
         states.add(robot.new DriveDistancePID(30, 180, 0.1, 2, 1));
         states.add(robot.new Wait(3));
         states.add(robot.new DriveDistancePID(310, 0, 0.5, 5, 1));

         //park
         states.add(robot.new TurnGyroPID(1, 90, 2, 1, false));
         states.add(robot.new DriveDistancePID(2200, 0, 1, 8, 1));
         //states.add(robot.new TurnGyroPID(1, 90, 2, 1, false));
         states.add(robot.new DriveDistancePID(650, 0, 1, 8, 1));

     }

     private void RedAutonomous2NoDuck(int delay)
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

     private void RedAutonomous1All()
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
         states.add(robot.new DriveDistancePID(200, 180, 0.75, 5, 1));

         //go to carousel
         states.add(robot.new DriveDistancePID( 800, 270, 0.8, 5, 1));
         states.add(robot.new DriveDistanceColor( robot.outerColorSensor, 270, 0.2, 1450, false));
         states.add(robot.new DriveDistancePID( 175, 270, 0.2, 5, 1));
         states.add(robot.new DriveDistanceColor(robot.outerColorSensor, 180, 0.2, 130, false));
         states.add(robot.new DriveDistancePID(70, 180, 0.5, 2, 1));
         states.add(robot.new DriveDistancePID(30, 180, 0.1, 2, 1));
         states.add(robot.new Wait(3));
         states.add(robot.new DriveDistancePID(310, 0, 0.5, 5, 1));

         //park
         states.add(robot.new TurnGyroPID(1, 90, 2, 1, false));
         states.add(robot.new DriveDistancePID(2200, 0, 1, 8, 1));
         states.add(robot.new TurnGyroPID(1, 90, 2, 1, false));
         states.add(robot.new DriveDistancePID(650, 0, 1, 8, 1));

     }

     private void RedAutonomous1NoDuck(int delay)
     {
         states = new ArrayList<State>();

         states.add(robot.new Wait(delay));

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
         states.add(robot.new DriveDistancePID(50, 180, 0.75, 5, 1));

         //park
         states.add(robot.new TurnGyroPID(1, 90, 2, 1, false));
         states.add(robot.new DriveDistancePID(1500, 0, 1, 8, 1));


     }
 }

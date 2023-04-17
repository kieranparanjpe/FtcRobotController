 package org.firstinspires.ftc.teamcode.PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Global.OdometryBot;
import org.firstinspires.ftc.teamcode.Global.SlidePosition;
import org.firstinspires.ftc.teamcode.Global.State;

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
         boolean right = true;
         boolean parking = true;
         int target = 2;

         //AutoNoCycle();
         ParkingSetUp();

         while(!isStarted())
         {
             if(gamepad1.dpad_right)
             {
                 cycle = true;
                 right = true;
                 parking = false;
                 FourConeRight();

                 chosen = "Auto: Four Right; Cycle: " + cycle;
             }

             if(gamepad1.dpad_left)
             {
                 cycle = true;
                 right = false;
                 parking = false;
                 FourConeLeft();


                 chosen = "Auto: Four Left; Cycle: " + cycle;
             }

             if(gamepad1.x)
             {
                 cycle = true;
                 right = false;
                 parking = false;
                 Order37Left();

                 chosen = "Order 37 Left" + cycle;
             }

             if(gamepad1.b)
             {
                 cycle = true;
                 right = true;
                 parking = false;
                 Order37Right();

                 chosen = "Order 37 Right" + cycle;
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
             int rightSideXOffset;
             if (parking) {
                 rightSideXOffset = 3000;
             } else {
                 rightSideXOffset = 10000;
             }
             if (target == 1) {
                 RobotLog.d("PARK 1");
                 if (right) {
                     states.add(robot.new AllenCoordDrive(-35000 + rightSideXOffset, -70000, 0, 1000, 2, 0.9, true, false));
                 } else {
                     states.add(robot.new AllenCoordDrive(-35000, -70000, 0, 1000, 2, 0.9, true, false));
                 }
                 states.add(robot.new Wait(delay));
                 states.add(robot.new Wait(delay));
                 states.add(robot.new Wait(delay));
                 states.add(robot.new Wait(delay));
             }
             if (target == 2) {
                 RobotLog.d("PARK 2");
                 if (right) {
                     states.add(robot.new AllenCoordDrive(-2000 + rightSideXOffset, -66000, 0, 1000, 2, 0.9, true, false));
                 } else {
                     states.add(robot.new AllenCoordDrive(-2000, -66000, 0, 1000, 2, 0.9, true, false));
                 }
                 states.add(robot.new Wait(delay));
                 states.add(robot.new Wait(delay));
                 states.add(robot.new Wait(delay));
                 states.add(robot.new Wait(delay));
             }
             if (target == 3) {
                 RobotLog.d("PARK 3");
                 if (right) {
                     states.add(robot.new AllenCoordDrive(27000 + rightSideXOffset, -70000, 0, 1000, 2, 0.9, true, false));
                 } else {
                     states.add(robot.new AllenCoordDrive(27000, -70000, 0, 1000, 2, 0.9, true, false));
                 }
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

     private void ParkingSetUp(){
         states = new ArrayList<State>();

//         states.add(robot.new AllenCoordDrive(-30000, -60000, 0, 40000, 2, 1, false, false));
//         states.add(robot.new Wait(0.3));
         states.add(robot.new AllenCoordDrive( 0, -67000, 0, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(5));
         states.add(robot.new AllenCoordDrive( 0, -67000, 0, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(2));
     }

     private void FourConeLeft()
     {
         double placeWait = 0.2;
         double pickupPositionX = -27000;
         double pickupPositionY = -68000;
         double dropAngle = 60;

         states = new ArrayList<State>();

         states.add(robot.new AllenCoordDrive(-30000, -60000, 0, 40000, 2, 1, false, false));
         states.add(robot.new Wait(0.3));
         states.add(robot.new AllenCoordDrive(-3000, -53000, 25, 500, 2, 1, true, false));

         states.add(robot.new SetSlidePosition(SlidePosition.HIGHAUTO));
         states.add(robot.new ArmToPosition(true));
         states.add(robot.new Wait(1.8));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGHAUTO));
         states.add(robot.new Wait(placeWait));
         states.add(robot.new SetClaw(true));
         states.add(robot.new SetSlidePosition(SlidePosition.DOWN4));
         states.add(robot.new Wait(0.5));
         //drive to 1st pickup
         states.add(robot.new AllenCoordDrive(-10000, pickupPositionY, 90, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(1));
         states.add(robot.new AllenCoordDrive(pickupPositionX, pickupPositionY, 90, 500, 2, 0.4, true, false));
         states.add(robot.new Wait(1.3));
         //1st pickup
         states.add(robot.new SetClaw(false));
         states.add(robot.new Wait(0.3));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGHAUTO));
         states.add(robot.new ArmToPosition(true));
         states.add(robot.new Wait(0.4));
         //drive to drop position
         states.add(robot.new AllenCoordDrive(-17000, -67000, dropAngle, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(1.5));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGHAUTO));
         states.add(robot.new Wait(placeWait));
         states.add(robot.new SetClaw(true));
         states.add(robot.new SetSlidePosition(SlidePosition.DOWN3));
         states.add(robot.new Wait(0.5));
         //drive to 2nd pickup
         states.add(robot.new AllenCoordDrive(-17000, -67000, 90, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(1.5));
         states.add(robot.new AllenCoordDrive(pickupPositionX, pickupPositionY, 90, 500, 2, 0.5, true, false));
         states.add(robot.new Wait(0.8));
         //2nd pickup
         states.add(robot.new SetClaw(false));
         states.add(robot.new Wait(0.2));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGHAUTO));
         states.add(robot.new ArmToPosition(true));
         states.add(robot.new Wait(0.4));
         //drive to drop position
         states.add(robot.new AllenCoordDrive(-17000, -67000, dropAngle, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(1.5));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGHAUTO));
         states.add(robot.new Wait(placeWait));
         states.add(robot.new SetClaw(true));
         states.add(robot.new SetSlidePosition(SlidePosition.DOWN2));
         states.add(robot.new Wait(0.5));
         //drive to 3rd pickup
         states.add(robot.new AllenCoordDrive(-17000, -67000, 90, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(1.5));
         states.add(robot.new AllenCoordDrive(pickupPositionX, pickupPositionY, 90, 500, 2, 0.5, true, false));
         states.add(robot.new Wait(0.8));
         //3rd pickup
         states.add(robot.new SetClaw(false));
         states.add(robot.new Wait(0.2));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGHAUTO));
         states.add(robot.new ArmToPosition(true));
         states.add(robot.new Wait(0.4));
         //drive to drop position
         states.add(robot.new AllenCoordDrive(-17000, -67000, dropAngle, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(1.5));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGHAUTO));
         states.add(robot.new Wait(placeWait));
         states.add(robot.new SetClaw(true));
         states.add(robot.new SetSlidePosition(SlidePosition.DOWN1));
         states.add(robot.new Wait(0.5));
         //drive to 4th pickup
         states.add(robot.new AllenCoordDrive(-17000, -67000, 90, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(1.5));
         states.add(robot.new AllenCoordDrive(pickupPositionX, pickupPositionY, 90, 500, 2, 0.5, true, false));
         states.add(robot.new Wait(0.8));
         //4th pickup
         states.add(robot.new SetClaw(false));
         states.add(robot.new Wait(0.2));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGHAUTO));
         states.add(robot.new ArmToPosition(true));
         states.add(robot.new Wait(0.4));
         //drive to drop position
         states.add(robot.new AllenCoordDrive(-17000, -67000, dropAngle, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(1.5));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGHAUTO));
         states.add(robot.new Wait(placeWait));
         states.add(robot.new SetClaw(true));
         states.add(robot.new SetSlidePosition(SlidePosition.DOWN));
         states.add(robot.new Wait(0.7));
         states.add(robot.new ArmToPosition(true));
     }

     private void FourConeRight()
     {
         double placeWait = 0.2;
         double pickupPositionX = 27000;
         double pickupPositionY = -68000;
         double dropAngle = -60;

         states = new ArrayList<State>();

         states.add(robot.new AllenCoordDrive(30000, -60000, 0, 40000, 2, 1, false, false));
         states.add(robot.new Wait(0.3));
         states.add(robot.new AllenCoordDrive(3000, -53000, -25, 500, 2, 1, true, false));

         states.add(robot.new SetSlidePosition(SlidePosition.HIGHAUTO));
         states.add(robot.new ArmToPosition(true));
         states.add(robot.new Wait(1.8));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGHAUTO));
         states.add(robot.new Wait(placeWait));
         states.add(robot.new SetClaw(true));
         states.add(robot.new SetSlidePosition(SlidePosition.DOWN4));
         states.add(robot.new Wait(0.5));
         //drive to 1st pickup
         states.add(robot.new AllenCoordDrive(10000, pickupPositionY, -90, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(1));
         states.add(robot.new AllenCoordDrive(pickupPositionX, pickupPositionY, -90, 500, 2, 0.4, true, false));
         states.add(robot.new Wait(1.3));
         //1st pickup
         states.add(robot.new SetClaw(false));
         states.add(robot.new Wait(0.3));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGHAUTO));
         states.add(robot.new ArmToPosition(true));
         states.add(robot.new Wait(0.9));
         //drive to drop position
         states.add(robot.new AllenCoordDrive(17000, -67000, dropAngle, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(1.5));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGHAUTO));
         states.add(robot.new Wait(placeWait));
         states.add(robot.new SetClaw(true));
         states.add(robot.new SetSlidePosition(SlidePosition.DOWN3));
         states.add(robot.new Wait(0.5));
         //drive to 2nd pickup
         states.add(robot.new AllenCoordDrive(17000, -67000, -90, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(1.5));
         states.add(robot.new AllenCoordDrive(pickupPositionX, pickupPositionY, -90, 500, 2, 0.5, true, false));
         states.add(robot.new Wait(0.8));
         //2nd pickup
         states.add(robot.new SetClaw(false));
         states.add(robot.new Wait(0.2));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGHAUTO));
         states.add(robot.new ArmToPosition(true));
         states.add(robot.new Wait(0.4));
         //drive to drop position
         states.add(robot.new AllenCoordDrive(17000, -67000, dropAngle, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(1.5));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGHAUTO));
         states.add(robot.new Wait(placeWait));
         states.add(robot.new SetClaw(true));
         states.add(robot.new SetSlidePosition(SlidePosition.DOWN2));
         states.add(robot.new Wait(0.5));
         //drive to 3rd pickup
         states.add(robot.new AllenCoordDrive(17000, -67000, -90, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(1.5));
         states.add(robot.new AllenCoordDrive(pickupPositionX, pickupPositionY, -90, 500, 2, 0.5, true, false));
         states.add(robot.new Wait(0.8));
         //3rd pickup
         states.add(robot.new SetClaw(false));
         states.add(robot.new Wait(0.2));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGHAUTO));
         states.add(robot.new ArmToPosition(true));
         states.add(robot.new Wait(0.4));
         //drive to drop position
         states.add(robot.new AllenCoordDrive(17000, -67000, dropAngle, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(1.5));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGHAUTO));
         states.add(robot.new Wait(placeWait));
         states.add(robot.new SetClaw(true));
         states.add(robot.new SetSlidePosition(SlidePosition.DOWN1));
         states.add(robot.new Wait(0.5));
         //drive to 4th pickup
         states.add(robot.new AllenCoordDrive(17000, -67000, -90, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(1.5));
         states.add(robot.new AllenCoordDrive(pickupPositionX, pickupPositionY, -90, 500, 2, 0.5, true, false));
         states.add(robot.new Wait(0.8));
         //4th pickup
         states.add(robot.new SetClaw(false));
         states.add(robot.new Wait(0.2));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGHAUTO));
         states.add(robot.new ArmToPosition(true));
         states.add(robot.new Wait(0.4));
         //drive to drop position
         states.add(robot.new AllenCoordDrive(17000, -67000, dropAngle, 1000, 2, 0.9, true, false));
         states.add(robot.new Wait(1.5));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGHAUTO));
         states.add(robot.new Wait(placeWait));
         states.add(robot.new SetClaw(true));
         states.add(robot.new SetSlidePosition(SlidePosition.DOWN));
         states.add(robot.new Wait(0.7));
         states.add(robot.new ArmToPosition(true));
     }

     private void Order37Left()
     {
         states = new ArrayList<State>();

         //first drive
         //  states.add(robot.new SetSlidePosition(SlidePosition.HIGH));


         states.add(robot.new AllenCoordDrive(-30000, -60000, 0, 40000, 2, 1, false, false));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGH));
         states.add(robot.new Wait(0.3));
         states.add(robot.new AllenCoordDrive(-3000, -53000, 25, 500, 2, 1, true, false));
         states.add(robot.new ArmToPosition(true));
         states.add(robot.new Wait(2));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGH));
         states.add(robot.new Wait(2));
         states.add(robot.new AllenCoordDrive(-3500, -52000, 25, 500, 2, 1, true, false));
         states.add(robot.new Wait(20));
         states.add(robot.new SetClaw(true));
         states.add(robot.new SetSlidePosition(SlidePosition.DOWN));
         states.add(robot.new Wait(0.5));
         //states.add(robot.new Wait(delay));
         states.add(robot.new ArmToPosition(true));
         states.add(robot.new Wait(1));
         states.add(robot.new AllenCoordDrive( -2000, -67000, 0, 500, 2, 0.4, true, false));
         states.add(robot.new Wait(1));
     }

     private void Order37Right()
     {
         states = new ArrayList<State>();

         //first drive
         //  states.add(robot.new SetSlidePosition(SlidePosition.HIGH));


         states.add(robot.new AllenCoordDrive(30000, -60000, 0, 40000, 2, 1, false, false));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGH));
         states.add(robot.new Wait(0.3));
         states.add(robot.new AllenCoordDrive(3000, -53000, -25, 500, 2, 1, true, false));
         states.add(robot.new ArmToPosition(true));
         states.add(robot.new Wait(2));
         states.add(robot.new SetSlidePosition(SlidePosition.HIGH));
         states.add(robot.new Wait(2));
         states.add(robot.new AllenCoordDrive(3500, -52000, -25, 500, 2, 1, true, false));
         states.add(robot.new Wait(20));
         states.add(robot.new SetClaw(true));
         states.add(robot.new SetSlidePosition(SlidePosition.DOWN));
         states.add(robot.new Wait(0.5));
         //states.add(robot.new Wait(delay));
         states.add(robot.new ArmToPosition(true));
         states.add(robot.new Wait(1));
         states.add(robot.new AllenCoordDrive( 2000, -67000, 0, 500, 2, 0.4, true, false));
         states.add(robot.new Wait(1));
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

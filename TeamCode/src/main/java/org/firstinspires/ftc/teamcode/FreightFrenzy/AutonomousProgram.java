 package org.firstinspires.ftc.teamcode.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Global.Robot;
import org.firstinspires.ftc.teamcode.Global.SlidePosition;
import org.firstinspires.ftc.teamcode.Global.State;

import java.util.ArrayList;

 @Disabled
@Autonomous(name = "Autonomous!", group = "Autonomous")
public class AutonomousProgram extends LinearOpMode {

    private Robot robot = null;

    private ArrayList<State> states = new ArrayList<State>();
    private ArrayList<State> asyncStates = new ArrayList<State>();

    int delay = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //region init
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        robot = new Robot(hardwareMap, this);


        String chosen = "";
        boolean duck = true;

        while(!isStarted())
        {
            if(gamepad1.right_trigger > 0)
                duck = true;
            if(gamepad1.left_trigger > 0)
                duck = false;

            if(gamepad1.a)
            {
                if(duck)
                    BlueAutonomous1All();
                else
                    BlueAutonomous1NoDuck(delay);

                chosen = "Blue 1; Ducks: " + duck;
            }

            if(gamepad1.b)
            {
                if(duck)
                    BlueAutonomous2All();
                else
                    BlueAutonomous2NoDuck(delay);

                chosen = "Blue 2; Ducks: " + duck;
            }

            if(gamepad1.x)
            {
                if(duck)
                    RedAutonomous1All();
                else
                    RedAutonomous1NoDuck(delay);

                chosen = "Red 1; Ducks: " + duck;
            }

            if(gamepad1.y)
            {
                if(duck)
                    RedAutonomous2All();
                else
                    RedAutonomous2NoDuck(delay);

                chosen = "Red 2; Ducks: " + duck;
            }

            telemetry.addData("Program to run: ", chosen);
            telemetry.update();
        }

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        telemetry.update();
        //endregion

        waitForStart();

        robot.runtime.reset();

        //robot.WebcamSetup(hardwareMap);



        int target = robot.GetTargetLocation();
        SlidePosition pos = SlidePosition.MID;
        if(target == 0)
            pos = SlidePosition.LOW;
        if(target == 1)
            pos = SlidePosition.MID;
        if(target == 2)
            pos = SlidePosition.HIGH;


        states.add(3, robot.new SetSlidePosition(pos));

        State currentState = states.get(0);
        int step = 0;

        double distanceToHub = 0;

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

            if(progress >= 1)
            {
                if(step == 5)
                {

                }

                step++;
                if(step >= states.size())
                    currentState = null;
                else
                    currentState = states.get(step);
            }

            robot.SlideToPosition(robot.slidePosition, 1);

            telemetry.addData("Distance driven ", distanceToHub);
            robot.outerColorSensor.Color();
            telemetry.addData("Step ", step);

            telemetry.addData("SLIDE", target);


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
        states.add(robot.new Dump(true));
        states.add(robot.new Wait(1.5f));
        states.add(robot.new Dump(false));
        states.add(robot.new Wait(0.75));
        states.add(robot.new SetSlidePosition(SlidePosition.DOWN));
        states.add(robot.new DriveDistancePID(150, 180, 0.75, 5, 1));

        //go to carousel
        states.add(robot.new TurnGyroPID(1, 270, 2, 1, false));
        states.add(robot.new DriveDistancePID( 800, 180, 0.8, 5, 1));
        states.add(robot.new DriveDistanceColor( robot.outerColorSensor, 180, 0.2, 1450, false));
        states.add(robot.new DriveDistancePID( 175, 180, 0.2, 5, 1));
        states.add(robot.new DriveDistanceColor(robot.outerColorSensor, 270, 0.2, 130, false));
        states.add(robot.new SpinCarousel(0.65, 14, true));
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

    private void BlueAutonomous2NoDuck(int delay)
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
        states.add(robot.new Dump(true));
        states.add(robot.new Wait(1.5f));
        states.add(robot.new Dump(false));
        states.add(robot.new Wait(0.75));
        states.add(robot.new SetSlidePosition(SlidePosition.DOWN));
        states.add(robot.new DriveDistancePID(150, 180, 0.75, 5, 1));

        //park
        states.add(robot.new TurnGyroPID(1, 270, 2, 1, false));
        states.add(robot.new DriveDistancePID(1500, 0, 1, 8, 1));

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
        states.add(robot.new Dump(true));
        states.add(robot.new Wait(1.5f));
        states.add(robot.new Dump(false));
        states.add(robot.new Wait(0.75));
        states.add(robot.new SetSlidePosition(SlidePosition.DOWN));
        states.add(robot.new DriveDistancePID(150, 180, 0.75, 5, 1));

        //go to carousel
        states.add(robot.new TurnGyroPID(1, 270, 2, 1, false));
        states.add(robot.new DriveDistancePID( 800, 180, 0.8, 5, 1));
        states.add(robot.new DriveDistanceColor( robot.outerColorSensor, 180, 0.2, 1450, false));
        states.add(robot.new DriveDistancePID( 175, 180, 0.2, 5, 1));
        states.add(robot.new DriveDistanceColor(robot.outerColorSensor, 270, 0.2, 130, false));
        states.add(robot.new SpinCarousel(0.65, 14, true));
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
        states.add(robot.new Dump(true));
        states.add(robot.new Wait(1.5f));
        states.add(robot.new Dump(false));
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
        states.add(robot.new Dump(true));
        states.add(robot.new Wait(1.5f));
        states.add(robot.new Dump(false));
        states.add(robot.new Wait(0.75));
        states.add(robot.new SetSlidePosition(SlidePosition.DOWN));
        states.add(robot.new DriveDistancePID(150, 180, 0.75, 5, 1));

        //go to carousel
        states.add(robot.new DriveDistancePID( 800, 270, 0.8, 5, 1));
        states.add(robot.new DriveDistanceColor( robot.outerColorSensor, 270, 0.2, 1450, false));
        states.add(robot.new DriveDistancePID( 175, 270, 0.2, 5, 1));
        states.add(robot.new DriveDistanceColor(robot.outerColorSensor, 180, 0.2, 130, false));
        states.add(robot.new SpinCarousel(-0.65, 14, true));
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
        states.add(robot.new Dump(true));
        states.add(robot.new Wait(1.5f));
        states.add(robot.new Dump(false));
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
        states.add(robot.new Dump(true));
        states.add(robot.new Wait(1.5f));
        states.add(robot.new Dump(false));
        states.add(robot.new Wait(0.75));
        states.add(robot.new SetSlidePosition(SlidePosition.DOWN));
        states.add(robot.new DriveDistancePID(200, 180, 0.75, 5, 1));

        //go to carousel
        states.add(robot.new DriveDistancePID( 800, 270, 0.8, 5, 1));
        states.add(robot.new DriveDistanceColor( robot.outerColorSensor, 270, 0.2, 1450, false));
        states.add(robot.new DriveDistancePID( 175, 270, 0.2, 5, 1));
        states.add(robot.new DriveDistanceColor(robot.outerColorSensor, 180, 0.2, 130, false));
        states.add(robot.new SpinCarousel(-0.65, 14, true));
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
        states.add(robot.new Dump(true));
        states.add(robot.new Wait(1.5f));
        states.add(robot.new Dump(false));
        states.add(robot.new Wait(0.75));
        states.add(robot.new SetSlidePosition(SlidePosition.DOWN));
        states.add(robot.new DriveDistancePID(50, 180, 0.75, 5, 1));

        //park
        states.add(robot.new TurnGyroPID(1, 90, 2, 1, false));
        states.add(robot.new DriveDistancePID(1500, 0, 1, 8, 1));


    }
}

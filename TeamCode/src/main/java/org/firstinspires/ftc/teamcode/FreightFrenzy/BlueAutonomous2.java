package org.firstinspires.ftc.teamcode.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Global.Robot;
import org.firstinspires.ftc.teamcode.Global.SlidePosition;

@Disabled
@Autonomous(name = "BlueAutonomous2", group = "Autonomous")
public class BlueAutonomous2 extends LinearOpMode {

    private Robot robot = null;

    private boolean bucketState;
    private double bucketTimer = 0.5, currentBucketTime;

    private double currentSlideTime = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        //region init
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        robot = new Robot(hardwareMap, this);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //endregion

        waitForStart();

        robot.runtime.reset();

        robot.targetBarcode = robot.GetTargetLocation();

        telemetry.addLine("Target Pos: " + robot.targetBarcode);
        telemetry.update();

        //start in front of shipping hub

        //can also use DriveDistance for slightly faster but less precise movements
        //Distance in mm, direction in degrees from forward (i dont remember if 90 is strafe left or right), dont use anything other than 0, 90, 180 or 270
        //Speed between 0-1
        //Timout is in ms just make it like 4000+

        robot.DriveDistancePID(50, 0, 0.5, 10000, 1);


        robot.DriveDistancePID(600, 90, 0.5, 10000, 1);


        robot.DriveDistancePID(500, 0, 0.5, 10000, 1);

        boolean complete = false;
        //goes to slide high position -> change .HIGH to other stuff (in autocomplete) for different heights

        if(robot.targetBarcode == 2)
            robot.slidePosition = SlidePosition.HIGH;
        if(robot.targetBarcode == 1)
            robot.slidePosition = SlidePosition.MID;
        if(robot.targetBarcode == 0)
            robot.slidePosition = SlidePosition.LOW;
 

        while(!complete && opModeIsActive())
        {
            complete = robot.SlideToPosition(robot.slidePosition, 1) == 1;

            if(complete)
            {
                if(currentSlideTime == 0.0)
                {
                    //Dumps (true means it will go to specified end position from robot.java)
                    if(robot.targetBarcode != 0)
                        robot.bucketServo.SetPosition(true);
                    currentSlideTime = robot.runtime.seconds();
                    complete = false;
                }
                else if(robot.runtime.seconds() - currentSlideTime < 2.0)
                {
                    complete = false;
                }
            }

            //just some debug stuff to phone screen
            telemetry.addLine("Slide Position (goal 1400ish): " + robot.slideMotor.getCurrentPosition());
            telemetry.update();
        }
        robot.slideMotor.setPower(0);

        //sleep(1000);

        //Reset Bucket
        robot.bucketServo.SetPosition(false);


        //goes to slide Down position -> change .DOWN to other stuff (in autocomplete) for different heights
        complete = false;
        robot.slidePosition = SlidePosition.DOWN;
        while(!complete && opModeIsActive())
        {
            complete = robot.SlideToPosition(robot.slidePosition, 1) == 1;
            //just some debug stuff to phone screen
            telemetry.addLine("Slide Position (goal 800ish): " + robot.slideMotor.getCurrentPosition());
            telemetry.update();
        }
        robot.slideMotor.setPower(0);

        //Backup a bit
        robot.DriveDistancePID(115, 180, 0.5, 1000, 1);

        //I dont remember if 90 is right or left (for red side which is what we have in basement). we want right for red left for blue
        robot.TurnGyroPID(0.5, 270, 3000, 1);

        //drive into depot over pipes
        robot.DriveDistance(2000, 0, 1, 3000);

        complete = false;
        robot.slidePosition = SlidePosition.DOWN;
        while(!complete && opModeIsActive())
        {
            complete = robot.SlideToPosition(robot.slidePosition, 1) == 1;
            //just some debug stuff to phone screen
            telemetry.addLine("Slide Position (goal 0): " + robot.slideMotor.getCurrentPosition());
            telemetry.update();
        }
        robot.slideMotor.setPower(0);
    }
}

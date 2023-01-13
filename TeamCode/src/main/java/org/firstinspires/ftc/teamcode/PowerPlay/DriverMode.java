package org.firstinspires.ftc.teamcode.PowerPlay;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Global.RobotPowerPlay;
import org.firstinspires.ftc.teamcode.Global.SlidePosition;
import org.firstinspires.ftc.teamcode.UltimateGoal.DriveControlState;

@Config
@TeleOp(name="Driver Control PP", group="TeleOp")
public class DriverMode extends LinearOpMode
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

        dashboard = FtcDashboard.getInstance();

        attachmentController = gamepad2;

        robot = new RobotPowerPlay(hardwareMap, this);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //endregion

        waitForStart();
       // robot.GetTargetLocation();

        robot.runtime.reset();

        while(opModeIsActive()) {

            //region driving
            //also had to comment out this to press play with sensors unplugged, remove when necessary
          /*  robot.bucketColorSensor.Color();

            //telemetry.addData("Distance A: ", robot.outerColorSensor.Distance());
            //telemetry.addData("Distance B: ", robot.bucketColorSensor.Distance());*/
            double driveSpeed = 0;

            if(gamepad1.dpad_left)
                attachmentController = gamepad1;

            if(attachmentController != gamepad1)
            {
                driveSpeed = gamepad1.left_trigger > 0 ? minDriveSpeed : gamepad1.right_trigger > 0 ? maxDriveSpeed : normalDriveSpeed;
            }
            else
                driveSpeed = gamepad1.right_stick_button ? minDriveSpeed : maxDriveSpeed;

            if(driveState == DriveControlState.DRIVING)
            {

               /* double driveTurn = -gamepad1.right_stick_x;
                //driveVertical = -gamepad1.right_stick_y;
                //driveHorizontal = gamepad1.right_stick_x;

                double gamepadXCoordinate = gamepad1.left_stick_x; //this simply gives our x value relative to the driver
                double gamepadYCoordinate = -gamepad1.left_stick_y; //this simply gives our y vaue relative to the driver
                double gamepadMagnitude = Range.clip(Math.hypot(gamepadXCoordinate, gamepadYCoordinate), 0, 1);
                //finds just how much power to give the robot based on how much x and y given by gamepad
                //range.clip helps us keep our power within positive 1
                // also helps set maximum possible value of 1/sqrt(2) for x and y controls if at a 45 degree angle (which yields greatest possible value for y+x)
                double gamepadDegree = Math.atan2(gamepadYCoordinate, gamepadXCoordinate);
                //the inverse tangent of opposite/adjacent gives us our gamepad degree
                double robotDegree = robot.imuData.HeadingAngleRAD();
                //gives us the angle our robot is at
                double movementDegree = gamepadDegree - robotDegree;
                telemetry.addData("Movement> degree", movementDegree);

                //adjust the angle we need to move at by finding needed movement degree based on gamepad and robot angles
                double gamepadXControl = Math.cos((movementDegree)) * gamepadMagnitude * driveSpeed;
                //by finding the adjacent side, we can get our needed x value to power our motors
                double gamepadYControl = Math.sin((movementDegree)) * gamepadMagnitude * driveSpeed;
                //by finding the opposite side, we can get our needed y value to power our motors


                telemetry.addData("GPX", gamepadXControl);
                telemetry.addData("GPY", gamepadYControl);

                double rightFrontPower = Range.clip((gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) + driveTurn), -driveSpeed, driveSpeed);
                double rightBackPower = Range.clip((gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) + driveTurn), -driveSpeed, driveSpeed);
                double leftFrontPower = Range.clip((gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) - driveTurn), -driveSpeed, driveSpeed);
                double leftBackPower = Range.clip((gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) - driveTurn), -driveSpeed, driveSpeed);

               /* double rightFrontPower = Range.clip((gamepadYControl - gamepadXControl + driveTurn), -driveSpeed, driveSpeed);
                double rightBackPower = Range.clip((gamepadYControl + gamepadXControl + driveTurn), -driveSpeed, driveSpeed);
                double leftFrontPower = Range.clip((gamepadYControl + gamepadXControl - driveTurn), -driveSpeed, driveSpeed);
                double leftBackPower = Range.clip((gamepadYControl - gamepadXControl - driveTurn), -driveSpeed, driveSpeed);*/


                //Normal Driving
                double leftY = -gamepad1.left_stick_y; //driving
                double leftX = gamepad1.left_stick_x;
                double rightX = gamepad1.right_stick_x; //turning


                double leftBackPower = Range.clip(leftY - leftX + rightX, -driveSpeed, driveSpeed);
                double leftFrontPower = Range.clip(leftY + leftX + rightX, -driveSpeed, driveSpeed);//problem left x
                double rightBackPower = Range.clip(leftY + leftX - rightX, -driveSpeed, driveSpeed);
                double rightFrontPower = Range.clip(leftY - leftX - rightX, -driveSpeed, driveSpeed);

                //telemetry.addLine("Power, " + leftBackPower);


                robot.SetPower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            }

            //endregion

            if(attachmentController.dpad_up)
                manualSlide = !manualSlide;




            //robot.slideMotor1.setPower(attachmentController.left_stick_y);
            //robot.slideMotor2.setPower(attachmentController.left_stick_y);

        //    robot.slideMotor1.setPower(0.1);
         //   robot.slideMotor2.setPower(0.1);

            if(attachmentController == gamepad2 && attachmentController.left_stick_y != 0)
            {
                robot.slideMotor1.setPower(attachmentController.left_stick_y);
                robot.slideMotor2.setPower(attachmentController.left_stick_y);
            }
            else
            {

                if(attachmentController.y)
                    robot.slidePosition = SlidePosition.HIGH;
                else if(attachmentController.a) {
                    robot.slidePosition = SlidePosition.DOWN;
                    robot.dropArm = true;
                }
                else if(attachmentController.b)
                    robot.slidePosition = SlidePosition.MID;
                else if(attachmentController.x)
                    robot.slidePosition = SlidePosition.LOW;
                else if(attachmentController.dpad_up)
                    robot.slidePosition = SlidePosition.GROUND;


                robot.SlideToPosition(robot.slidePosition, 1);
            }

            if(attachmentController == gamepad2 && attachmentController.right_stick_y != 0)
            {
                robot.armServo1.ChangePosition(attachmentController.right_stick_y * 0.01);
                robot.armServo2.ChangePosition(attachmentController.right_stick_y * 0.01);

            }
            else
            {
                robot.ArmToPosition();
            }

            if(attachmentController.right_trigger > 0 && robot.runtime.seconds() - currentPincherTimer > pincherTimer)
            {
                currentPincherTimer = robot.runtime.seconds();
                pincherState = !pincherState;

                robot.pincherServo.SetPosition(pincherState);
            }

            //endregion
            telemetry.addLine("Slide Pos: " + robot.slideMotor2.getCurrentPosition());
            telemetry.addLine("Angle: " + robot.imuData.HeadingAngle());
            telemetry.addData("Drive Type", driveState.toString());
            telemetry.addData("Runtime", robot.runtime.time());
            telemetry.update();
        }

        
        //Drive forward
    }


}


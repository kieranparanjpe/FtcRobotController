package org.firstinspires.ftc.teamcode.FreightFrenzy;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Global.CappingPosition;
import org.firstinspires.ftc.teamcode.Global.Robot;
import org.firstinspires.ftc.teamcode.Global.SlidePosition;
import org.firstinspires.ftc.teamcode.UltimateGoal.DriveControlState;

@Disabled
@Config
@TeleOp(name="Driver Control FF", group="TeleOp")
public class DriverControl extends LinearOpMode
{
    //change to gamepad2 for 2 drivers
    private Gamepad attachmentController;

    private Robot robot = null;

    private DriveControlState driveState = DriveControlState.DRIVING;


    private boolean manualSlide = false;
    private boolean bucketState;
    private boolean hasCap = false;
    private double bucketTimer = 0.5, currentBucketTime;
    private double cappingTimer = 0.5, currentCappingTime;

    private double maxDriveSpeed = 1;
    private double normalDriveSpeed = 0.4;
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

        robot = new Robot(hardwareMap, this);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //endregion

        waitForStart();
        robot.GetTargetLocation();

        robot.runtime.reset();

        while(opModeIsActive()) {

            //region driving
            //also had to comment out this to press play with sensors unplugged, remove when necessary
          /*  robot.bucketColorSensor.Color();

            //telemetry.addData("Distance A: ", robot.outerColorSensor.Distance());
            //telemetry.addData("Distance B: ", robot.bucketColorSensor.Distance());*/
            robot.outerColorSensor.Color();

            double driveSpeed = 0;

            if(attachmentController != gamepad1)
            {
                driveSpeed = gamepad1.left_trigger > 0 ? minDriveSpeed : gamepad1.right_trigger > 0 ? maxDriveSpeed : normalDriveSpeed;
            }
            else
                driveSpeed = gamepad1.right_stick_button ? minDriveSpeed : maxDriveSpeed;

            if(driveState == DriveControlState.DRIVING)
            {
                double leftY = -gamepad1.left_stick_y; //driving
                double leftX = gamepad1.left_stick_x;
                double rightX = gamepad1.right_stick_x; //turning


                double leftBackPower = Range.clip(leftY - leftX + rightX, -driveSpeed, driveSpeed);
                double leftFrontPower = Range.clip(leftY + leftX + rightX, -driveSpeed, driveSpeed);
                double rightBackPower = Range.clip(leftY + leftX - rightX, -driveSpeed, driveSpeed);
                double rightFrontPower = Range.clip(leftY - leftX - rightX, -driveSpeed, driveSpeed);

                //telemetry.addLine("Power, " + leftBackPower);

                robot.SetPower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            }

            //endregion

            robot.intakeMotor.setPower(-attachmentController.right_trigger - attachmentController.left_trigger / 2);

            if(attachmentController.dpad_up)
                manualSlide = !manualSlide;

            if(manualSlide)
            {
                if(attachmentController.right_bumper)
                    robot.slideMotor.setPower(1);
                else if(attachmentController.left_bumper)
                    robot.slideMotor.setPower(-1);
                else
                    robot.slideMotor.setPower(0);
            }
            else
            {
                if(attachmentController.right_bumper)
                    robot.slidePosition = SlidePosition.HIGH;
                else if(attachmentController.left_bumper)
                    robot.slidePosition = SlidePosition.DOWN;
                else if(attachmentController.y)
                    robot.slidePosition = SlidePosition.MID;

                robot.SlideToPosition(robot.slidePosition, 1);
            }

            if(attachmentController.a && robot.runtime.seconds() - currentBucketTime > bucketTimer && robot.slidePosition != SlidePosition.DOWN)
            {
                currentBucketTime = robot.runtime.seconds();
                bucketState = !bucketState;

                robot.bucketServo.SetPosition(bucketState);
            }

            if(attachmentController.x)
                robot.carouselMotor.setPower(0.7);
            else if (attachmentController.b)
                robot.carouselMotor.setPower(-0.7);
            else
                robot.carouselMotor.setPower(0);

            if(manualSlide)
            {
                if(attachmentController.right_bumper)
                    robot.slideMotor.setPower(1);
                else if(attachmentController.left_bumper)
                    robot.slideMotor.setPower(-1);
                else
                    robot.slideMotor.setPower(0);
            }
            else
            {
                if(attachmentController.right_bumper)
                    robot.slidePosition = SlidePosition.HIGH;
                else if(attachmentController.left_bumper)
                    robot.slidePosition = SlidePosition.DOWN;
                else if(attachmentController.y)
                    robot.slidePosition = SlidePosition.MID;

                robot.SlideToPosition(robot.slidePosition, 1);
            }

            if(attachmentController.left_stick_y != 0)
            {
                //robot.cappingMotor.setDirection(attachmentController.left_stick_y < 0 ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
                robot.cappingMotor.setVelocity(attachmentController.left_stick_y * directDriveSpeed);

            }
            else
            {
                if(attachmentController.right_stick_y > 0.5)
                    robot.cappingPosition = CappingPosition.PICKING;
                else if(attachmentController.right_stick_y < -0.5)
                    robot.cappingPosition = CappingPosition.HIGH;
                else if (Math.abs(attachmentController.right_stick_x) > 0.5)
                    robot.cappingPosition = CappingPosition.DOWN;

                robot.CappingArmToPosition(robot.cappingPosition, hasCap);
            }


            if(attachmentController.left_stick_button && robot.runtime.seconds() - currentCappingTime > cappingTimer)
            {
                currentCappingTime = robot.runtime.seconds();
                hasCap = !hasCap;
            }

            //endregion
            telemetry.addLine("TSE in possession? " + hasCap);
            telemetry.addLine("Cap position " + robot.cappingMotor.getCurrentPosition());
            telemetry.addLine("Angle: " + robot.imuData.HeadingAngle());
            telemetry.addData("Drive Type", driveState.toString());
            telemetry.addData("Runtime", robot.runtime.time());
            telemetry.update();
        }

        
        //Drive forward
    }


}


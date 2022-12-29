package org.firstinspires.ftc.teamcode.UltimateGoal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Global.IMUData;
import org.firstinspires.ftc.teamcode.Global.ServoData;
@Disabled

@TeleOp(name="DriverControl", group="TeleOp")
public class DriverControl extends RobotFunctions
{
    private ElapsedTime runtime = new ElapsedTime();

    private DriveBaseData driveBaseData = null;

    private IMUData imuData = null;

    private DcMotorEx intakeMotor = null;
    private DcMotorEx shooterMotor = null;
    private DcMotorEx wobbleMotor = null;

    private ServoData wobbleServo = null;
    private ServoData ringServoArm = null;

    private DriveControlState driveState = DriveControlState.DRIVING;
    private WobbleState wobbleState = WobbleState.NOTHING;
    private boolean wobbleGrab;
    private boolean shooterOn;
    private boolean ringFlick = false;

    private double lastRingFlick;
    private int ringFlickTime = 1500; // -> change this value (in ms) to change how quickly you can load the shooter.

    private final double highGoalDegreesPerSecond = 0.85;
    private final double powerShotDegreesPerSecond = 0.75;
    private double flyWheelSpeed;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //region init
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        driveBaseData = new DriveBaseData("LeftFront","RightFront","LeftBack", "RightBack", 75, 1440, hardwareMap);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "ShooterMotor");
        wobbleMotor = hardwareMap.get(DcMotorEx.class, "WobbleMotor");

        wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wobbleServo = new ServoData("WobbleServoArm", 0.0, 0.5, hardwareMap, Servo.Direction.FORWARD);
        ringServoArm = new ServoData("RingServoArm", 0.0, 0.2, hardwareMap, Servo.Direction.FORWARD);

        flyWheelSpeed = highGoalDegreesPerSecond;

        imuData = new IMUData("imu", hardwareMap);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //endregion

        waitForStart();

        runtime.reset();

        while(opModeIsActive()) {

            //region driving

            if(gamepad1.dpad_up)
                driveState = DriveControlState.TURNING;

            if(driveState == DriveControlState.DRIVING)
            {
                if(gamepad1.x)
                    driveState = DriveControlState.GRID;

                double leftY = -gamepad1.left_stick_y; //driving
                double leftX = gamepad1.left_stick_x;
                double rightX = gamepad1.right_stick_x; //turning

                double leftBackPower = Range.clip(leftY - leftX + rightX, -1.0, 1.0);
                double leftFrontPower = Range.clip(leftY + leftX + rightX, -1.0, 1.0);
                double rightBackPower = Range.clip(leftY + leftX - rightX, -1.0, 1.0);
                double rightFrontPower = Range.clip(leftY - leftX - rightX, -1.0, 1.0);

                driveBaseData.SetPower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            }
            else if(driveState == DriveControlState.GRID)
            {
                if(gamepad1.x)
                    driveState = DriveControlState.DRIVING;

                double leftY = -gamepad1.left_stick_y; //driving
                double leftX = gamepad1.left_stick_x;
                double rightX = gamepad1.right_stick_x; //turning

                double leftBackPower = 0;
                double leftFrontPower = 0;
                double rightBackPower = 0;
                double rightFrontPower = 0;

                if(leftY > 0.3)
                {
                    leftBackPower = Range.clip(1 + rightX, -1.0, 1.0);
                    leftFrontPower = Range.clip(1 + rightX, -1.0, 1.0);
                    rightBackPower = Range.clip(1 - rightX, -1.0, 1.0);
                    rightFrontPower = Range.clip(1 - leftX - rightX, -1.0, 1.0);
                }
                else if(leftY < -0.3)
                {
                    leftBackPower = Range.clip(-1 + rightX, -1.0, 1.0);
                    leftFrontPower = Range.clip(-1 + rightX, -1.0, 1.0);
                    rightBackPower = Range.clip(-1 - rightX, -1.0, 1.0);
                    rightFrontPower = Range.clip(-1 - leftX - rightX, -1.0, 1.0);
                }
                else if(leftX > 0.3)
                {
                    leftBackPower = Range.clip(-1 + rightX, -1.0, 1.0);
                    leftFrontPower = Range.clip(1 + rightX, -1.0, 1.0);
                    rightBackPower = Range.clip(1 - rightX, -1.0, 1.0);
                    rightFrontPower = Range.clip(-1 - leftX - rightX, -1.0, 1.0);
                }
                else if(leftX < -0.3)
                {
                    leftBackPower = Range.clip(1 + rightX, -1.0, 1.0);
                    leftFrontPower = Range.clip(-1 + rightX, -1.0, 1.0);
                    rightBackPower = Range.clip(-1 - rightX, -1.0, 1.0);
                    rightFrontPower = Range.clip(1 - leftX - rightX, -1.0, 1.0);
                }
                else
                {
                    leftBackPower = 0;
                    leftFrontPower = 0;
                    rightBackPower = 0;
                    rightFrontPower = 0;
                }

                driveBaseData.SetPower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            }
            else if(driveState == DriveControlState.TURNING)
            {
                TurnGyro(driveBaseData, 0.4, 0, imuData, 4);
                driveState = DriveControlState.DRIVING;
            }
            //endregion

            //region shooter/intake

            if(gamepad1.dpad_down)
                flyWheelSpeed = flyWheelSpeed == highGoalDegreesPerSecond ? powerShotDegreesPerSecond : highGoalDegreesPerSecond;

            if(gamepad1.right_trigger > 0.1)
                shooterOn = !shooterOn;
            //region adjustments
            /*if(gamepad1.left_stick_button)
                flyWheelSpeed -= 0.0001;
            if(gamepad1.right_stick_button)
                flyWheelSpeed += 0.0001;*/
            //endregion

            if(shooterOn/*gamepad1.right_trigger > 0.1*/)
                shooterMotor.setPower(flyWheelSpeed);
            else
                shooterMotor.setPower(0);

            int intakePower = 0;

            if (gamepad1.right_bumper)
                intakePower = -1;
            else if (gamepad1.left_bumper)
                intakePower = 1;

            intakeMotor.setPower(intakePower);

            //endregion
            //region ring flick
            if (gamepad1.left_trigger > 0 && !ringFlick)
            {
                ringFlick = true;
                if(flyWheelSpeed == highGoalDegreesPerSecond)
                    ShootRings();
                else if(flyWheelSpeed == powerShotDegreesPerSecond)
                    ShootRingsPower();
                //lastRingFlick = runtime.milliseconds();
                //ringFlick = true;
            }

            /*if(lastRingFlick + (ringFlickTime / 2) < runtime.milliseconds())
                ringFlick = false;
            if (ringFlick)
                SetServoPosition(ringServoArm.servo, ringServoArm.targetPosition);
            else
                SetServoPosition(ringServoArm.servo, ringServoArm.startPosition);*/

            //endregion

            //region Wobble

            if(gamepad1.b)
                wobbleGrab = !wobbleGrab;

            if(gamepad1.a)
                wobbleMotor.setPower(0.1);
            else if(gamepad1.y)
                wobbleMotor.setPower(-0.1);
            else
                wobbleMotor.setPower(0);

            if (wobbleGrab)
                SetServoPosition(wobbleServo.servo, wobbleServo.targetPosition);
            else
                SetServoPosition(wobbleServo.servo, wobbleServo.startPosition);

            //endregion
            telemetry.addLine("Angle: " + imuData.HeadingAngle());
            telemetry.addData("Shooter Speed", (flyWheelSpeed == highGoalDegreesPerSecond ? "High Goal" : "Power Shot") + " Actual Speed: " + flyWheelSpeed);
            telemetry.addData("Drive Type", driveState.toString());
            telemetry.addData("Runtime", runtime.time());
            telemetry.update();
        }

        //Drive forward
    }

    public void ShootRings()
    {
        for (int i = 0; i < 3; i++)
        {
            SetServoPosition(ringServoArm.servo, ringServoArm.targetPosition);
            sleep(ringFlickTime);
            SetServoPosition(ringServoArm.servo, ringServoArm.startPosition);
            sleep(ringFlickTime);
        }

        ringFlick = false;

    }

    private void ShootRingsPower()
    {
        SetServoPosition(ringServoArm.servo, ringServoArm.targetPosition);
        sleep(1500);
        //DriveLeftRightDistance(driveBaseData, 0.5, 190, 4);
        SetServoPosition(ringServoArm.servo, ringServoArm.startPosition);
        TurnGyro(driveBaseData, 0.25, -15, imuData, 3);

        SetServoPosition(ringServoArm.servo, ringServoArm.targetPosition);
        sleep(1500);
        //DriveLeftRightDistance(driveBaseData, 0.5, 190, 4);
        SetServoPosition(ringServoArm.servo, ringServoArm.startPosition);
        TurnGyro(driveBaseData, 0.25, -20, imuData, 3);

        SetServoPosition(ringServoArm.servo, ringServoArm.targetPosition);
        sleep(1500);
        SetServoPosition(ringServoArm.servo, ringServoArm.startPosition);
        TurnGyro(driveBaseData, 0.25, 0, imuData, 3);

        shooterMotor.setPower(0);


       /* for (int i = 0; i < 3; i++)
        {
            //TurnGyro(driveBaseData, 0.25, 0, imuData, 3);

            SetServoPosition(ringServoArm.servo, ringServoArm.targetPosition);
            sleep(1500);
            if(i < 2)
            {
                //DriveLeftRightDistance(driveBaseData, 0.5, 190, 4);
                SetServoPosition(ringServoArm.servo, ringServoArm.startPosition);

                sleep(ringFlickTime - 500);
            }
            else
            {

                //TurnGyro(driveBaseData, 0.25, 0, imuData, 3);
                sleep(1000);

                SetServoPosition(ringServoArm.servo, ringServoArm.startPosition);
            }

        }*/

    }
}

package org.firstinspires.ftc.teamcode.Global;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.core.Scalar;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Hashtable;
import java.util.Locale;

@Config
public  class RobotPowerPlay {

    public ElapsedTime runtime = new ElapsedTime();

    private final double  encoderTicksPerRevolution = 751.6;
    private final double wheelCircumference = Math.PI * 96; //given in mm
    private final double encoderTickPerMM = encoderTicksPerRevolution / wheelCircumference;

    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightBack = null;

    private IMUData imu;
    private LinearOpMode linearOpMode;

    public IMUData imuData = null;

    public DcMotorEx slideMotor1;
    public DcMotorEx slideMotor2;
    public ServoData armServo1;
    public ServoData armServo2;
    public ServoData pincherServo;

    public ColorSensor slideColourSensor;

    public PIDController rightSlidePID = new PIDController(0.001, 0.00, 0.0002);//0.01, 0, 0.0000 ; 0.0002
    public PIDController leftSlidePID = new PIDController(0.001, 0.00, 0.0002);//0.01, 0, 0.0000 ; 0.0002

    private PIDController lFPID = new PIDController(0.001, 0.000, 0.0000);
    private PIDController rFPID = new PIDController(0.001, 0.000, 0.0000);
    private PIDController lBPID = new PIDController(0.001, 0.000, 0.0000);
    private PIDController rBPID = new PIDController(0.001, 0.000, 0.000);
    private GyroPIDController gyroPID = new GyroPIDController(0.5, 0, 0);

    private final double maxSlideSpeed = 1;
    private final double pincherLightOpen = 0.275;
    private final double pincherHardOpen = 0.2;

    public SlidePosition slidePosition = SlidePosition.DOWN;
    private final Hashtable<SlidePosition, Integer> slidePositions = new Hashtable<SlidePosition, Integer>();
    private final Hashtable<SlidePosition, Double> armPositions = new Hashtable<SlidePosition, Double>();
    public boolean dropArm = false;


    private WebcamData webcamData;
    private OpenCVPipeline openCVPipeline;
    WebcamAprilTags webcamPipeline;

    private final int hueLow = 167, hueHigh = 235, satLow = 128, satHigh = 255, valLow = 60, valHigh = 255; // colour ranges, 0 - 255
    private final int leftX = 0, topY = 350, rightX = 1920, bottomY = 850; //coords in picture to look at (doesnt need to process the edges. eg: dont check the walls)
    private final int leftThreshHold = 650, rightThreshHold = 1300;
    public int targetBarcode = 2;

    // region Init

    public RobotPowerPlay(HardwareMap hardwareMap, LinearOpMode opMode)
    {
        linearOpMode = opMode;
        InitHardware(hardwareMap);
        SetSlidePositions();
    }

    private void InitHardware(HardwareMap hardwareMap)
    {
        leftFront = hardwareMap.get(DcMotorEx.class, "LeftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "RightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "LeftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "RightBack");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        SetMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        SetMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        slideColourSensor = new ColorSensor(hardwareMap.get(RevColorSensorV3.class, "SlideSensor"),
                new ColourRange[] { new ColourRange(new Scalar(140, 100, 0), new Scalar(260, 260, 260)), new ColourRange(new Scalar(0, 100, 0), new Scalar(40, 260, 260))}

                , linearOpMode.telemetry);


        slideMotor1 = hardwareMap.get(DcMotorEx.class, "SlideMotor1");
        slideMotor2 = hardwareMap.get(DcMotorEx.class, "SlideMotor2");

        pincherServo = new ServoData("GripServo", 0.33, pincherHardOpen, hardwareMap, Servo.Direction.FORWARD);
        //had to comment below cuz it wouldn't let me press play with the sensors unplugged, can change when necessary
       // outerColorSensor = new ColorSensor(hardwareMap.get(RevColorSensorV3.class, "OuterSensor"),
         //       new ColourRange[] { new ColourRange(new Scalar(140, 100, 0), new Scalar(260, 260, 260)), new ColourRange(new Scalar(0, 100, 0), new Scalar(40, 260, 260))}
            //    , linearOpMode.telemetry);
        //mess with these values for capping

        armServo1 = new ServoData("4Bar1", 0.4, 0.35, hardwareMap, Servo.Direction.REVERSE);
        armServo2 = new ServoData("4Bar2", 0.4, 0.35, hardwareMap, Servo.Direction.FORWARD);

        slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        imuData = new IMUData("imu", hardwareMap);

       webcamPipeline = new WebcamAprilTags(hardwareMap, linearOpMode.telemetry);
        //webcamData = new WebcamData(hardwareMap, linearOpMode.telemetry);
        /*if(linearOpMode.getClass() != DriverControl.class)
            targetBarcode = WebcamSetup(hardwareMap);*/
    }

    private void SetSlidePositions()
    {
        slidePositions.put(SlidePosition.DOWN, 0);
        slidePositions.put(SlidePosition.DOWN1, 0);
        slidePositions.put(SlidePosition.DOWN2, 0);
        slidePositions.put(SlidePosition.DOWN3, 0);
        slidePositions.put(SlidePosition.DOWN4, 0);
        slidePositions.put(SlidePosition.GROUND, 67); //100
        slidePositions.put(SlidePosition.LOW, 733); //1100
        slidePositions.put(SlidePosition.MID, 1332); //2000
        slidePositions.put(SlidePosition.HIGH, 2064); // 3100


        armPositions.put(SlidePosition.DOWN, 0.86);
        armPositions.put(SlidePosition.DOWN1, 0.83);
        armPositions.put(SlidePosition.DOWN2, 0.79);
        armPositions.put(SlidePosition.DOWN3, 0.76);
        armPositions.put(SlidePosition.DOWN4, 0.70);

        armPositions.put(SlidePosition.GROUND, 0.5);
        armPositions.put(SlidePosition.LOW, 0.12); //400
        armPositions.put(SlidePosition.MID, 0.2); //800
        armPositions.put(SlidePosition.HIGH, 0.2);

    }

    //endregion

    //region Basic

    public void SetPower(double lf, double rf, double lb, double rb)
    {
        leftFront.setVelocity(encoderTicksPerRevolution * 5.2 * lf);
        rightFront.setVelocity(encoderTicksPerRevolution * 5.2 * rf);
        leftBack.setVelocity(encoderTicksPerRevolution * 5.2 * lb);
        rightBack.setVelocity(encoderTicksPerRevolution * 5.2 * rb);
    }

    public void SetMode(DcMotor.RunMode runMode)
    {
        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
        leftBack.setMode(runMode);
        rightBack.setMode(runMode);
    }

    public void SetTargetPosition(int positionLF, int positionRF, int positionLB, int positionRB)
    {
        leftFront.setTargetPosition(positionLF);
        rightFront.setTargetPosition(positionRF);
        leftBack.setTargetPosition(positionLB);
        rightBack.setTargetPosition(positionRB);
    }

    //endregion

    //region Advanced Linear

    //direction given in degrees where 0 is straight ahead, 90 is to the right and -90 is to the left
    public void DriveTime(int milliseconds, double direction, double speed)
    {
        VectorData v = VectorData.ComputePower(direction, speed);
        SetPower(v.x, v.y, v.y, v.x);

        linearOpMode.sleep(milliseconds);

        SetPower(0, 0, 0, 0);
    }

    //distance in mm
    public void DriveDistance(double distance, double direction, double speed, double timeoutRedundancy)
    {
        SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        VectorData v = VectorData.ComputePower(direction, distance);

        //VectorData distanceM = VectorData.SetMag(distance, v);

        int newPositionLeftFront = (int) (Math.round(v.x * encoderTickPerMM));
        int newPositionRightFront = (int) (Math.round(v.y * encoderTickPerMM));
        int newPositionLeftBack = (int) (Math.round(v.y * encoderTickPerMM));
        int newPositionRightBack = (int) (Math.round(v.x * encoderTickPerMM));

        int pos = leftFront.getCurrentPosition();

        SetTargetPosition(newPositionLeftFront, newPositionRightFront, newPositionLeftBack, newPositionRightBack);

        SetMode(DcMotor.RunMode.RUN_TO_POSITION);

        SetPower(speed, speed, speed, speed);

        runtime.reset();

        while (linearOpMode.opModeIsActive() && runtime.time() < timeoutRedundancy &&
                (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {
            pos = leftFront.getCurrentPosition();
            linearOpMode.telemetry.addData("Motor", pos);
            linearOpMode.telemetry.addLine("Motors: Running");

            linearOpMode.telemetry.update();
        }
        linearOpMode.telemetry.addData("Motor", pos);
        linearOpMode.telemetry.addLine("Motors: Complete");

        linearOpMode.telemetry.update();

        SetPower(0, 0, 0 ,0);

        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);


        linearOpMode.sleep(100);
    }
    public void DriveDistancePID(double distance, double direction, double speed, double timeoutRedundancy, double accuracy)
    {
        SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);

        VectorData v = VectorData.ComputePower(direction, distance);
        int newPositionLeftFront = (int) (Math.round(v.x * encoderTickPerMM));
        int newPositionRightFront = (int) (Math.round(v.y * encoderTickPerMM));
        int newPositionLeftBack = (int) (Math.round(v.y * encoderTickPerMM));
        int newPositionRightBack = (int) (Math.round(v.x * encoderTickPerMM));

        boolean done = false;

        do
        {
            double lfPower = Range.clip(lFPID.Compute(newPositionLeftFront, leftFront.getCurrentPosition()), -speed, speed);
            double rfPower = Range.clip(rFPID.Compute(newPositionRightFront, rightFront.getCurrentPosition()), -speed, speed);
            double lbPower = Range.clip(lBPID.Compute(newPositionLeftBack, leftBack.getCurrentPosition()), -speed, speed);
            double rbPower = Range.clip(rBPID.Compute(newPositionRightBack, rightBack.getCurrentPosition()), -speed, speed);

            SetPower(lfPower, rfPower, lbPower, rbPower);
            done = ((lFPID.Completed(accuracy) + rFPID.Completed(accuracy) + lBPID.Completed(accuracy) + rBPID.Completed(accuracy)) / 4) == 1;

        }
        while(linearOpMode.opModeIsActive() && !done && runtime.time() < timeoutRedundancy);

        linearOpMode.telemetry.addLine("New Position " + leftFront.getCurrentPosition());

        linearOpMode.telemetry.update();

        SetPower(0, 0, 0 ,0);

        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
    public void TurnGyro(double power, double angle, double timeoutRedundancy)
    {
        linearOpMode.sleep(200);

        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentAngle = imuData.HeadingAngle();
        angle = Range.clip(angle, -180, 180);

        if(angle > currentAngle)
        {
            SetPower(-power, power, -power, power);
            runtime.reset();

            while (linearOpMode.opModeIsActive() && runtime.time() < timeoutRedundancy && angle >= currentAngle + 10)
            {
                currentAngle = imuData.HeadingAngle();
                linearOpMode.telemetry.addLine("Motors: Running");
                linearOpMode.telemetry.addLine("Current Angle: " + currentAngle);

                linearOpMode.telemetry.update();
            }
        }
        else if(angle < currentAngle)
        {
            SetPower(power, -power, power, -power);
            runtime.reset();

            while (linearOpMode.opModeIsActive() && runtime.time() < timeoutRedundancy && angle <= currentAngle - 10)
            {
                currentAngle = imuData.HeadingAngle();
                linearOpMode.telemetry.addLine("Motors: Running");
                linearOpMode.telemetry.addLine("Current Angle: " + currentAngle);

                linearOpMode.telemetry.update();
            }
        }

        linearOpMode.telemetry.addLine("Motors: Complete");

        linearOpMode.telemetry.update();

        SetPower(0, 0, 0, 0);
        linearOpMode.sleep(100);
    }

    public void TurnGyroPID(double maxSpeed, double angle, double timeoutRedundancy, double accuracy)
    {
        boolean done = false;
        do
        {
            double current = imuData.HeadingAngle();

            linearOpMode.telemetry.addLine("Current Heading " + current);

            double turn = gyroPID.Compute(Math.toRadians(-angle), Math.toRadians(current));
            turn = Range.clip(turn, -maxSpeed, maxSpeed);

            SetPower(-turn, turn, -turn, turn);

            done = gyroPID.Completed(accuracy) == 1;
            linearOpMode.telemetry.update();

        }
        while(linearOpMode.opModeIsActive() && !done && runtime.time() < timeoutRedundancy);
        linearOpMode.telemetry.addLine("Done turn " + gyroPID.derivative);
        linearOpMode.telemetry.addLine("Current Heading " + imuData.HeadingAngle());

        linearOpMode.telemetry.update();

        SetPower(0, 0, 0 ,0);
    }

    //endregion

    //region Advanced State Machine
    //distance in mm
    public class DriveDistance extends State
    {
        double distance;
        double direction;
        double speed;
        double timeoutRedundancy;

        public DriveDistance(double distance, double direction, double speed, double timeoutRedundancy, boolean async)
        {
            runAsync = async;
            this.distance = distance;
            this.direction = direction;
            this.speed = speed;
            this.timeoutRedundancy = timeoutRedundancy;
        }

        @Override
        public double Run() {
            if (progress == 0) {
                progress = 0.01;
                timeStarted = runtime.seconds();
                SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                VectorData v = VectorData.ComputePower(direction, distance);

                //VectorData distanceM = VectorData.SetMag(distance, v);

                int newPositionLeftFront = (int) (Math.round(v.x * encoderTickPerMM));
                int newPositionRightFront = (int) (Math.round(v.y * encoderTickPerMM));
                int newPositionLeftBack = (int) (Math.round(v.y * encoderTickPerMM));
                int newPositionRightBack = (int) (Math.round(v.x * encoderTickPerMM));

                SetTargetPosition(newPositionLeftFront, newPositionRightFront, newPositionLeftBack, newPositionRightBack);

                SetMode(DcMotor.RunMode.RUN_TO_POSITION);

                SetPower(speed, speed, speed, speed);
            } else if (progress < 1) {
                if (linearOpMode.opModeIsActive() && runtime.seconds() - timeStarted < timeoutRedundancy && rightBack.isBusy()) {
                    int pos = leftFront.getCurrentPosition();
                    progress = pos / leftFront.getTargetPosition();
                    linearOpMode.telemetry.addData("Motor", pos);
                    linearOpMode.telemetry.addLine("Motors: Running");

                    linearOpMode.telemetry.update();
                } else {
                    progress = 1.5;
                }
            }
            if (progress >= 1) {
                linearOpMode.telemetry.addLine("Motors: Complete");

                linearOpMode.telemetry.update();

                SetPower(0, 0, 0, 0);

                SetMode(DcMotor.RunMode.RUN_USING_ENCODER);


                linearOpMode.sleep(100);
            }
            return progress;
        }
    }

    public class DriveDistanceColor extends State
    {
        ColorSensor sensor;
        double direction;
        double speed;
        double dis;
        double distanceMax;
        public double distance;

        boolean hub;

        public DriveDistanceColor(ColorSensor sensor, double direction, double speed, double distanceMax, boolean async)
        {
            runAsync = async;
            this.sensor = sensor;
            this.direction = direction;
            this.speed = speed;
            this.distanceMax = distanceMax;
        }

        @Override
        public double Run() {
            if (progress == 0) {
                progress = 0.01;
                timeStarted = runtime.seconds();
                SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                VectorData v = VectorData.ComputePower(direction, 1);

                //VectorData distanceM = VectorData.SetMag(distance, v);

                SetMode(DcMotor.RunMode.RUN_USING_ENCODER);

                SetPower(v.x * speed, v.y * speed, v.y * speed, v.x * speed);
            } else if (progress < 1) {
                if (linearOpMode.opModeIsActive() && distance < distanceMax && !sensor.Detected()) {
                    int pos = leftFront.getCurrentPosition();
                    linearOpMode.telemetry.addData("Motor", pos);
                    linearOpMode.telemetry.addLine("Motors: Running");
                    distance = pos / encoderTickPerMM;
                    linearOpMode.telemetry.update();
                } else {
                    progress = 1.5;
                }
            }
            if (progress >= 1) {
                linearOpMode.telemetry.addLine("Motors: Complete");

                linearOpMode.telemetry.update();

                SetPower(0, 0, 0, 0);

                SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
                int pos = leftFront.getCurrentPosition();
                distance = pos / encoderTickPerMM;

                linearOpMode.sleep(100);
            }
            return progress;
        }
    }

    public class TurnColor extends State
    {
        ColorSensor sensor;
        double direction;
        double speed;
        double distanceMax;
        public double distance;
        double timeoutRedundancy;



        //direction: 1 is right, -1 is left
        public TurnColor(ColorSensor sensor, double direction, double speed, double timeoutRedundancy, boolean async)
        {
            runAsync = async;
            this.sensor = sensor;
            this.direction = direction;
            this.speed = speed;
            this.timeoutRedundancy = timeoutRedundancy;
        }

        @Override
        public double Run() {
            linearOpMode.telemetry.addData("sensor distance: ", sensor.Distance());
            if (progress == 0) {
                progress = 0.01;
                timeStarted = runtime.seconds();
                SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                //VectorData distanceM = VectorData.SetMag(distance, v);

                SetMode(DcMotor.RunMode.RUN_USING_ENCODER);


                SetPower(speed * direction, -speed * direction, speed*direction, -speed * direction);
            } else if (progress < 1) {
                if (linearOpMode.opModeIsActive() && runtime.seconds() - timeStarted < timeoutRedundancy && sensor.Distance() > 0) {
                    int pos = leftFront.getCurrentPosition();
                    linearOpMode.telemetry.addData("Motor", pos);
                    linearOpMode.telemetry.addLine("Motors: Running");
                    //linearOpMode.telemetry.update();
                } else {
                    progress = 1.5;
                }
            }
            if (progress >= 1) {
                linearOpMode.telemetry.addLine("Motors: Complete");

                linearOpMode.telemetry.update();

                SetPower(0, 0, 0, 0);

                SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
                int pos = leftFront.getCurrentPosition();
                distance = pos / encoderTickPerMM;

                linearOpMode.sleep(100);
            }
            return progress;
        }
    }

    public class DriveDistanceDist extends State
    {
        ColorSensor sensor;
        double direction;
        double speed;
        double dis;
        double distanceMax;
        public double distance;

        boolean hub;

        public DriveDistanceDist(ColorSensor sensor, double direction, double speed, double distanceMax, boolean async)
        {
            runAsync = async;
            this.sensor = sensor;
            this.direction = direction;
            this.speed = speed;
            this.distanceMax = distanceMax;
        }

        @Override
        public double Run() {
            if (progress == 0) {
                progress = 0.01;
                timeStarted = runtime.seconds();
                SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                VectorData v = VectorData.ComputePower(direction, 1);

                //VectorData distanceM = VectorData.SetMag(distance, v);

                SetMode(DcMotor.RunMode.RUN_USING_ENCODER);

                SetPower(v.x * speed, v.y * speed, v.y * speed, v.x * speed);
            } else if (progress < 1) {
                if (linearOpMode.opModeIsActive() && distance < distanceMax && sensor.Distance() < sensor.distance) {
                    int pos = leftFront.getCurrentPosition();
                    linearOpMode.telemetry.addData("Motor", pos);
                    linearOpMode.telemetry.addLine("Motors: Running");
                    distance = pos / encoderTickPerMM;
                    linearOpMode.telemetry.update();
                } else {
                    progress = 1.5;
                }
            }
            if (progress >= 1) {
                linearOpMode.telemetry.addLine("Motors: Complete");

                linearOpMode.telemetry.update();

                SetPower(0, 0, 0, 0);

                SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
                int pos = leftFront.getCurrentPosition();
                distance = pos / encoderTickPerMM;

                linearOpMode.sleep(100);
            }
            return progress;
        }
    }

    public class DriveDistancePID extends State
    {
        double distance;
        double direction;
        double speed;
        double timeoutRedundancy;
        double accuracy;

        int newPositionLeftFront;
        int newPositionRightFront;
        int newPositionLeftBack;
        int newPositionRightBack;

        private PIDController lFPID = new PIDController(0.001, 0.000, 0.0000);
        private PIDController rFPID = new PIDController(0.001, 0.000, 0.0000);
        private PIDController lBPID = new PIDController(0.001, 0.000, 0.0000);
        private PIDController rBPID = new PIDController(0.001, 0.000, 0.000);

        double startAngle = 0;

        public DriveDistancePID(double distance, double direction, double speed, double timeoutRedundancy, double accuracy)
        {
            this.distance = distance;
            this.direction = direction;
            this.speed = speed;
            this.timeoutRedundancy = timeoutRedundancy;
            this.accuracy = accuracy;
            this.runAsync = false;
        }

        @Override
        public double Run() {
            if (progress == 0) {
                progress = 0.01;
                timeStarted = runtime.seconds();
                SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                SetMode(DcMotor.RunMode.RUN_USING_ENCODER);

                VectorData v = VectorData.ComputePower(direction, distance);
                newPositionLeftFront = (int) (Math.round(v.x * encoderTickPerMM));
                newPositionRightFront = (int) (Math.round(v.y * encoderTickPerMM));
                newPositionLeftBack = (int) (Math.round(v.y * encoderTickPerMM));
                newPositionRightBack = (int) (Math.round(v.x * encoderTickPerMM));
                startAngle = imuData.HeadingAngle();
            } else if (progress < 1) {
                if (linearOpMode.opModeIsActive() ) {
                    double lfPower = Range.clip(lFPID.Compute(newPositionLeftFront, leftFront.getCurrentPosition()), -speed, speed);
                    double rfPower = Range.clip(rFPID.Compute(newPositionRightFront, rightFront.getCurrentPosition()), -speed, speed);
                    double lbPower = Range.clip(lBPID.Compute(newPositionLeftBack, leftBack.getCurrentPosition()), -speed, speed);
                    double rbPower = Range.clip(rBPID.Compute(newPositionRightBack, rightBack.getCurrentPosition()), -speed, speed);

                    SetPower(lfPower, rfPower, lbPower, rbPower);

                    progress = (lFPID.Completed() + rFPID.Completed() + lBPID.Completed() + rBPID.Completed()) / 4;
                } else {
                    progress = 1.5;
                }
            }
            if (progress >= 1) {

                linearOpMode.telemetry.addLine("New Position " + leftFront.getCurrentPosition());

                linearOpMode.telemetry.update();

                SetPower(0, 0, 0 ,0);

                SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            return progress;
        }
    }

    public class TurnGyroPID extends State
    {
        double angle;
        double accuracy;
        double maxSpeed;
        double timeoutRedundancy;

        public TurnGyroPID(double maxSpeed, double angle, double timeoutRedundancy, double accuracy, boolean async)
        {
            runAsync = async;
            this.angle = angle;
            this.accuracy = accuracy;
            this.maxSpeed = maxSpeed;
            this.timeoutRedundancy = timeoutRedundancy;
        }

        @Override
        public double Run() {
            if (progress == 0) {
                progress = 0.01;
                timeStarted = runtime.seconds();

            } else if (progress < 1) {
                if ((linearOpMode.opModeIsActive() && runtime.seconds() - timeStarted < timeoutRedundancy)) {
                    double current = imuData.HeadingAngle();

                    linearOpMode.telemetry.addLine("Current Heading " + current);

                    double turn = gyroPID.Compute(Math.toRadians(-angle), Math.toRadians(current));
                    turn = Range.clip(turn, -maxSpeed, maxSpeed);

                    SetPower(-turn, turn, -turn, turn);

                    progress = gyroPID.Completed();
                    linearOpMode.telemetry.update();
                } else {
                    progress = 1.5;
                }
            }
            if (progress >= 1) {
                linearOpMode.telemetry.addLine("Done turn " + gyroPID.derivative);
                linearOpMode.telemetry.addLine("Current Heading " + imuData.HeadingAngle());

                linearOpMode.telemetry.update();

                SetPower(0, 0, 0 ,0);
            }
            return progress;
        }
    }

    public class Wait extends State
    {
        double seconds;

        public Wait(double seconds)
        {
            this.seconds = seconds;
        }

        @Override
        public double Run() {

            if (progress == 0) {
                progress = 0.01;
                timeStarted = runtime.seconds();

            } else if (progress < 1 || runtime.seconds() - timeStarted <= seconds) {
                progress = (runtime.seconds() - timeStarted) / seconds;
            }
            return progress;
        }
    }

    public class SetSlidePosition extends State
    {
        public SlidePosition position;

        public SetSlidePosition(SlidePosition position)
        {
            this.position = position;
        }

        @Override
        public double Run() {
            dropArm = true;
            slidePosition = position;
            return 1.5;
        }
    }

    public class ArmToPosition extends State
    {
        boolean state;
        double time;

        public ArmToPosition(boolean state)
        {
            dropArm = false;
            this.state = state;
            this.runAsync = false;
        }

        @Override
        public double Run() {
            dropArm = false;
            armServo1.SetPosition(state);
            armServo2.SetPosition(state);
            return 1.5;

        }
    }

    public class SetClaw extends State
    {
        public boolean state;

        public SetClaw(boolean position)
        {
            this.state = position;
        }

        @Override
        public double Run() {
            pincherServo.SetPosition(state);
            return 1.5;
        }
    }

    //endregion

    //region Attachments

    public double SlideToPosition(SlidePosition target, double accuaracy)
    {
        int targetHeight = slidePositions.get(slidePosition);
        int current = (slideMotor1.getCurrentPosition() + slideMotor2.getCurrentPosition())/2;

        if(targetHeight >= current || armServo1.servo.getPosition() > armServo1.startPosition - 0.05) {

            double powerRight = rightSlidePID.Compute(slidePositions.get(target), slideMotor2.getCurrentPosition());
            double powerLeft = leftSlidePID.Compute(slidePositions.get(target), slideMotor1.getCurrentPosition());

            if (Math.abs(powerRight) < 0.02)
                powerRight = 0;
            if (Math.abs(powerLeft) < 0.02)
                powerLeft = 0;

            linearOpMode.telemetry.addLine("Right Slide Position " + slideMotor2.getCurrentPosition());
            linearOpMode.telemetry.addLine("Left Slide Position " + slideMotor1.getCurrentPosition());


            slideMotor1.setPower(Range.clip(powerLeft, -maxSlideSpeed, maxSlideSpeed));
            slideMotor2.setPower(Range.clip(powerRight, -maxSlideSpeed, maxSlideSpeed));
        }

        return (leftSlidePID.Completed(accuaracy) + rightSlidePID.Completed(accuaracy)) / 2;
    }

    public void ArmToPosition()
    {
        if(!dropArm)
            return;

        int targetHeight = slidePositions.get(slidePosition);
        int current = (slideMotor1.getCurrentPosition() + slideMotor2.getCurrentPosition())/2;


       // if(slideMotor2.getCurrentPosition() < 800 && current >= 800)
           // targetHeight = slideMotor2.getCurrentPosition();
        
        double target = 0;
        
        if(targetHeight > current)
        {
            if(current < targetHeight - 150)
            {
                target = armServo1.startPosition;
            }
            else
                target = armPositions.get(slidePosition);
        }

        if(targetHeight <= current)
        {
            if(current > targetHeight + 300)
                target = armServo1.startPosition;
            else
                target = armPositions.get(slidePosition);

        }
        
        
     /*   if(targetHeight < slidePositionBounds[0])
        {
            target = armPositions[0];
            if(pincherServo.currentPosition == pincherLightOpen)
            {
                pincherServo.targetPosition = pincherHardOpen;
                pincherServo.SetPosition(true);
            }
        }
        else if(targetHeight >= slidePositionBounds[0] && current < slidePositionBounds[1]) {
            target = armPositions[1];
            if(pincherServo.currentPosition == pincherHardOpen)
            {
                pincherServo.targetPosition = pincherLightOpen;
                pincherServo.SetPosition(true);
            }
        }
        else if(targetHeight >= slidePositionBounds[1] && targetHeight < slidePositionBounds[2]) {
            target = armPositions[2];
            if(pincherServo.currentPosition == pincherHardOpen)
            {
                pincherServo.targetPosition = pincherLightOpen;
                pincherServo.SetPosition(true);
            }
        }
        else {
            target = armPositions[3];
            if(pincherServo.currentPosition == pincherHardOpen)
            {
                pincherServo.targetPosition = pincherLightOpen;
                pincherServo.SetPosition(true);
            }

        }*/
        armServo1.SetPosition(target);
        armServo2.SetPosition(target);
    }




    public double SlideToPosition(double change, double accuaracy)
    {
        //if(target == SlidePosition.DOWN)
        //bucketServo.SetPosition(false);

        double powerRight = rightSlidePID.Compute((int)(slideMotor2.getCurrentPosition() + change), slideMotor2.getCurrentPosition());
        double powerLeft = leftSlidePID.Compute((int)(slideMotor1.getCurrentPosition() + change), slideMotor1.getCurrentPosition());

        if(Math.abs(powerRight) < 0.02)
            powerRight = 0;
        if(Math.abs(powerLeft) < 0.02)
            powerLeft = 0;

        linearOpMode.telemetry.addLine("Right Slide Position " + slideMotor2.getCurrentPosition());
        linearOpMode.telemetry.addLine("Left Slide Position " + slideMotor1.getCurrentPosition());


        slideMotor1.setPower(Range.clip(powerLeft, -maxSlideSpeed, maxSlideSpeed));
        slideMotor2.setPower(Range.clip(powerRight, -maxSlideSpeed, maxSlideSpeed));

        return (leftSlidePID.Completed(accuaracy) + rightSlidePID.Completed(accuaracy)) / 2;
    }




    //endregion

    //region Webcam

    //should be called in a loop during init
    public int GetTargetLocation()
    {
        int r = webcamPipeline.GetTargetZone();

        return r;
    }

    public void StopWebcam()
    {
        webcamPipeline.Stop();
    }


    public int WebcamSetup(HardwareMap hardwareMap)
    {
        int target = 0;
        //Webcam stuff
        webcamData = new WebcamData(hardwareMap, linearOpMode.telemetry);

        /*
        double seconds = runtime.seconds();
        while(runtime.seconds() - seconds < 3){
            linearOpMode.telemetry.addLine("waiting");
            linearOpMode.telemetry.update();
        }
        */

        linearOpMode.sleep(1500);

        Bitmap picture = webcamData.GetImage();
        saveBitmap(picture);

        webcamData.CloseCamera();

        int counter = 1;
        int sum = 0;
        for (int x = leftX; x < rightX; x+=2) {
            for (int y = topY; y < bottomY; y+=5) {
                float[] HSV = new float[3];
                Color.colorToHSV(picture.getPixel(x, y), HSV);

                HSV[0] = (HSV[0] / 360) * 255;
                HSV[1] *= 255;
                HSV[2] *= 255;

                if(HSV[0] > hueLow && HSV[0] < hueHigh && HSV[1] > satLow && HSV[1] < satHigh && HSV[2] > valLow && HSV[2] < valHigh)
                {
                    sum += x;
                    counter++;
                }
            }
        }

        double avgPos = sum / counter;

        if(avgPos < leftThreshHold)
            target = 0;
        if(avgPos >= leftThreshHold && avgPos <= rightThreshHold)
            target = 1;
        if(avgPos > rightThreshHold)
            target = 2;

        if(picture == null)
            linearOpMode.telemetry.addData("Status", "Failure: Webcam was not able to get an image, try reinitializing");
        else
            linearOpMode.telemetry.addData("Status", "Initialized, you can now press play");

        linearOpMode.telemetry.addLine("Target Pixel Counter: " + counter);
        linearOpMode.telemetry.addLine("Target Pixel Pos: " + avgPos);
        linearOpMode.telemetry.addLine("Targeted Square: " + target + " (0 = A, 1 = B, 2 = C)");


        targetBarcode = target;
        return target;
    }

    private File captureDirectory = AppUtil.ROBOT_DATA_DIR;
    private int captureCounter;

    private void saveBitmap(Bitmap bitmap) {
        if(bitmap == null)
        {
            return;
        }

        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);


        File file = new File(captureDirectory, String.format(Locale.getDefault(), "webcam-frame-%d.jpg", captureCounter++));
        try {
            try (FileOutputStream outputStream = new FileOutputStream(file)) {
                bitmap.compress(Bitmap.CompressFormat.JPEG, 100, outputStream);
                linearOpMode.telemetry.log().add("captured %s", file.getName());
            }
        } catch (IOException e) {
            linearOpMode.telemetry.addLine("exception saving %s" + file.getName());
            linearOpMode.telemetry.update();
        }
    }
    //endregion
}

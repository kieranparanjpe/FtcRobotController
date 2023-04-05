package org.firstinspires.ftc.teamcode.Global;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

import java.io.OutputStreamWriter;

public class OdometryBot extends RobotPowerPlay {

   // public DcMotor rightFront = null;
    public DcMotor verticalRight = null;
    private DcMotor verticalLeft;

    //public Servo odometryRaise = null;

    String verticalLeftEncoderName = "DeadWheelLeft", verticalRightEncoderName = "DeadWheelRight",
            horizontalEncoderName = "RightFront";

    public double xBlue = 0, yBlue = 0, xBlueChange = 0, yBlueChange = 0, thetaDEG = 0, thetaRAD = 0;
    double xRed = 0, yRed = 0, xRedChange = 0, yRedChange = 0;
    double hError = 0;

    protected double[] driveAccelerationCurve = new double[]{0.5, 0.6, 0.8, 0.9, 0.8, 0.9};

    double savedXBlue, savedYBlue, savedThetaDEG;
    public double savedStartAngle;

    private int gyroResetCounter = 0;

    final int vLDirection = 1;
    final int vRDirection = 1;
    final int hDirection = -1;
    final double diameter = 5278; // actually diameter: 98/609 = d/32800
    final double hDiameter = 808; //diameter of horizontal encoder: 15/609 = hD/32800      24302
    final double leftX = -(diameter/2); //134mm
    final double rightX = (diameter/2); //152mm
    final double hY = -(hDiameter/2); //137mm

    double vLOffset, vROffset, hOffset = 0;

    public double previousVL = 0, previousVR = 0, previousH = 0, previousThetaRAD;
    double angleChange = 0;

    double drive;
    double strafe;
    double twist;
    double driveAngle;
    double thetaDifference;
    double distanceToTarget;
    long startTime;
    long elapsedTime = 0;
    public boolean isCoordinateDriving = false;
    public boolean isTurningInPlace = false;

    MiniPID drivePID = new MiniPID(0.1, 0, 0);//i: 0.006 d: 0.06
    MiniPID twistPID = new MiniPID(0.015, 0, 0);

    double globalTargetX = 0;
    double globalTargetY = 0;
    double globalTargetTheta = 0;
    int globalTolerance = 0;
    double globalAngleTol = 0;
    double globalMagnitude = 0;

    ElapsedTime robotLogTimer = new ElapsedTime();

    OutputStreamWriter odometryWriter;

    public OdometryBot(HardwareMap map, LinearOpMode opMode) {
        super(map, opMode);
        init(map, opMode);
    }

    public void init(HardwareMap ahwMap, LinearOpMode opMode) {
        initDriveHardwareMap(ahwMap);
        context = ahwMap.appContext;
        opMode.telemetry.addData("Status", "Init Complete");
        opMode.telemetry.update();
        robotLogTimer.reset();
    }

    private void initDriveHardwareMap(HardwareMap ahwMap){

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       verticalLeft = ahwMap.dcMotor.get(verticalLeftEncoderName);
       verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalRight = ahwMap.dcMotor.get(verticalRightEncoderName);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linearOpMode.telemetry.addData("Status", "Hardware Map Init Complete");
        linearOpMode.telemetry.update();
    }

    Context context;

    public void calculateCaseThree(double vL, double vR, double h) {
        vL = vL * vLDirection;
        vR = vR * vRDirection;
        h = h * hDirection;

        double lC = vL - previousVL;
        double rC = vR - previousVR;

//        linearOpMode.telemetry.addData("lC", lC);
//        linearOpMode.telemetry.addData("rC", rC);

        //angleChange = ((lC - rC) / (Math.PI * diameter * 2) * 360);
        //angleChange = (lC - rC)/(rightX - leftX);
        //linearOpMode.telemetry.addData("angleChange", angleChange);
//        angleChange = (lC - rC)/(2 * diameter);
//
//        angleDEG = angleDEG + angleChange;
        //thetaRAD = thetaRAD - angleChange;
        thetaRAD = -imuData.HeadingAngleRAD();
        thetaDEG = Math.toDegrees(thetaRAD);

        //thetaDEG = getDeltaAngle();

        angleChange = thetaRAD - previousThetaRAD;

        hError = (angleChange * hDiameter) / 2;

        //linearOpMode.telemetry.addData("hError", hError);

        double hC = h - previousH;
        //linearOpMode.telemetry.addData("hC", hC);

        xRedChange = hC - hError;
        //xRedChange = hC - (hY * angleChange);
        yRedChange = (lC + rC)/2;
        //yRedChange = ((lC * rightX) - (rC * leftX))/(rightX - leftX);

        xBlueChange = Math.cos(-thetaRAD - (Math.PI/2)) * xRedChange + Math.cos(-thetaRAD) * yRedChange;
        yBlueChange = Math.sin(-thetaRAD) * yRedChange + Math.sin(-thetaRAD - (Math.PI/2)) * xRedChange;
//        xBlueChange = xRedChange * Math.cos(thetaRAD) + yRedChange * Math.sin(thetaRAD);
//        yBlueChange = yRedChange * Math.cos(thetaRAD) + xRedChange * Math.sin(thetaRAD);

        xBlue = xBlue + yBlueChange;
        yBlue = yBlue + xBlueChange;
//        xBlue = xBlue + xBlueChange;
//        yBlue = yBlue + yBlueChange;

        previousVL = vL;
        previousVR = vR;
        previousH = h;
        previousThetaRAD = thetaRAD;

//        if (gyroResetCounter >= 9) {
//            reAngle(0);
//            gyroResetCounter = 0;
//        }

        gyroResetCounter++;
    }

    public void reAngle(double offset) {
            thetaRAD = -imuData.HeadingAngleRAD() + offset;
    }

//    public void resetOdometry(boolean button) {
//
//        if (button) {
////            vLOffset = leftFront.getCurrentPosition();
////            vROffset = rightFront.getCurrentPosition();
////            hOffset = horizontal.getCurrentPosition() + 79000;
//
//            horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//            previousVL = 0;
//            previousVR = 0;
//            previousH = 0;
//
//            xBlue = 79000;
//            yBlue = 0;
//
//            thetaDEG = 0;
//        }
//    }

    public class AllenCoordDrive extends State
    {
        TrajectorySequence myTrajectory;
        double timeoutRedundancy;

        double xTarget; double yTarget; double targetTheta; int tolerance; double angleTol; double magnitude; boolean brake;

        public AllenCoordDrive(double xTarget, double yTarget, double targetTheta, int tolerance, double angleTol, double magnitude, boolean brake, boolean async)
        {
            runAsync = async;
            this.xTarget = xTarget;
            this.yTarget = yTarget;
            this.targetTheta = targetTheta;
            this.tolerance = tolerance;
            this.angleTol = angleTol;
            this.magnitude = magnitude;
            this.brake = brake;
        }

        @Override
        public double Run() {
            if (progress == 0) {
                driveToCoordinate(xTarget, yTarget, targetTheta, tolerance, angleTol, magnitude, brake);

                progress = 0.01;
                timeStarted = runtime.seconds();
                //  SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else if (progress < 1) {
                if (linearOpMode.opModeIsActive()
                        && driveToCoordinateUpdate(globalTargetX, globalTargetY, globalTargetTheta, globalTolerance, globalAngleTol, globalMagnitude)) {

                } else {
                    progress = 1.5;
                }
            }
            if (progress >= 1) {
                linearOpMode.telemetry.addLine("Motors: Complete");

                linearOpMode.telemetry.update();
            }
            return progress;
        }
    }

    public void onTick(){
        //RobotLog.d(String.format("Position, heading: %.2f, %.2f, %.2f", xBlue, yBlue, thetaDEG));

        linearOpMode.telemetry.addData("X:", xBlue);
        linearOpMode.telemetry.addData("Y:", yBlue);
        linearOpMode.telemetry.addData("Theta:", thetaDEG);
        linearOpMode.telemetry.addData("vL", verticalLeft.getCurrentPosition());
        linearOpMode.telemetry.addData("vR", verticalRight.getCurrentPosition());
        linearOpMode.telemetry.addData("h", rightFront.getCurrentPosition());
        //opMode.telemetry.addData("h diameter", (int)((thetaDEG*360)/(horizontal.getCurrentPosition() * Math.PI)));
//        opMode.telemetry.update();

        //outputEncoders();
        //thetaDEG = -getDeltaAngle();
        calculateCaseThree(verticalLeft .getCurrentPosition() - vLOffset, -verticalRight.getCurrentPosition() - vROffset, -rightFront.getCurrentPosition() - hOffset);
        if (isCoordinateDriving) {
            driveToCoordinateUpdate(globalTargetX, globalTargetY, globalTargetTheta, globalTolerance, globalAngleTol, globalMagnitude);
        }
    }

    public void driveToCoordinate(double xTarget, double yTarget, double targetTheta, int tolerance, double angleTol, double magnitude, boolean brake) {
        if (brake) {
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        if (xBlue > xTarget) {
            distanceToTarget = - Math.sqrt(Math.pow(xBlue - xTarget, 2) + Math.pow(yBlue - yTarget, 2));
        } else {
            distanceToTarget = Math.sqrt(Math.pow(xBlue - xTarget, 2) + Math.pow(yBlue - yTarget, 2));
        }
        //RobotLog.d(String.format("BlueX: %f BlueY: %f Theta: %f", xBlue, yBlue, thetaDEG));
        globalTargetX = xTarget;
        globalTargetY = yTarget;
        globalTargetTheta = targetTheta;
        globalAngleTol = angleTol;
        globalTolerance = tolerance;
        globalMagnitude = magnitude;

        isCoordinateDriving = true;

//            driveToCoordinateUpdate(xTarget, yTarget, targetTheta, tolerance, magnitude);

//            elapsedTime = System.currentTimeMillis() - startTime;
//            if (elapsedTime > 10000) {
//                break;
//            }
    }

    public void driveToCoordinate(double xTarget, double yTarget, double targetTheta, int tolerance, double magnitude, boolean brake) {
        driveToCoordinate(xTarget, yTarget, targetTheta, tolerance, 1, magnitude, brake);
    }

    public boolean driveToCoordinateUpdate(double xTarget, double yTarget, double targetTheta, int tolerance, double angleTol, double bigMagnitude) {
        drivePID.setOutputLimits(bigMagnitude);
        twistPID.setOutputLimits(0.6);
        thetaDifference = targetTheta - thetaDEG;
        twist = twistPID.getOutput(thetaDEG, targetTheta);
        double rawDriveAngle = -Math.toDegrees(Math.atan2(xTarget - xBlue, yTarget - yBlue));
        driveAngle = (rawDriveAngle - thetaDEG);
        double magnitude = Math.min(bigMagnitude, Math.abs(drivePID.getOutput(distanceToTarget/3000, 0))*2);
        if (Math.abs(distanceToTarget) < 8000) {
            magnitude = Math.max(0.15, Math.min(1.0, Math.abs(drivePID.getOutput(distanceToTarget/800, 0))));
            magnitude = Range.clip(Math.abs(drivePID.getOutput(distanceToTarget/2000, 0)), 0.15, 0.5);

        }
        if (xBlue > xTarget) {
            distanceToTarget = - Math.sqrt(Math.pow(xBlue - xTarget, 2) + Math.pow(yBlue - yTarget, 2));
        } else {
            distanceToTarget = Math.sqrt(Math.pow(xBlue - xTarget, 2) + Math.pow(yBlue - yTarget, 2));
        }
        drive = (Math.cos(Math.toRadians(driveAngle)) * magnitude);
        strafe = Range.clip(Math.sin(Math.toRadians(driveAngle)) * magnitude * 1.3, 0, bigMagnitude);

        driveByVector(-drive, -strafe, twist, 1);
        RobotLog.d(String.format("BlueX: %f BlueY: %f Theta: %f Angle: %f Drive: %f Strafe: %f Twist: %f", xBlue, yBlue, thetaDEG, driveAngle, drive, strafe, twist));
        RobotLog.d(String.format("Distance: %f Magnitude: %f", distanceToTarget, magnitude));

        if ((xTarget + tolerance > xBlue) && (xTarget - tolerance < xBlue) && (yTarget + tolerance > yBlue) && (yTarget - tolerance < yBlue) && Math.abs(thetaDifference) < angleTol) {
            driveByVector(0, 0, 0, 1);
            RobotLog.d("TARGET REACHED");
            isCoordinateDriving = false;
            return true;

        } else {
            isCoordinateDriving = true;
            return false;

        }
    }

    public void driveByVector(double drive, double strafe, double twist, double multiplier) {
        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };

        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }
        //RobotLog.d(String.format("multiplier: %f speeds 0: %f", multiplier, speeds[0]));
        // apply the calculated values to the motors.
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearOpMode.telemetry.addData("lF", speeds[0] * multiplier);
        linearOpMode.telemetry.addData("rF", speeds[1] * multiplier);
        linearOpMode.telemetry.addData("lB", speeds[2] * multiplier);
        linearOpMode.telemetry.addData("rB", speeds[3] * multiplier);
        leftFront.setPower(speeds[0] * multiplier);
        rightFront.setPower(speeds[1] * multiplier);
        leftBack.setPower(speeds[2] * multiplier);
        rightBack.setPower(speeds[3] * multiplier);
    }

    public void waitForCoordinateDrive() {
        while (linearOpMode.opModeIsActive() && isCoordinateDriving) {
          //  sleep(0, "wait for drive");
        }
    }

//    public void savePosition() {
////        try {
////            odometryWriter = new FileWriter("/sdcard/FIRST/odometry positions.txt", false);
////        } catch (IOException e) {
////            throw new RuntimeException("odometry file writer open failed: " + e.toString());
////        }
////        try {
////            RobotLog.d("odometryWriter.write");
////            odometryWriter.write(xBlue + "\n");
////            odometryWriter.write(yBlue + "\n");
////            odometryWriter.write(thetaDEG + "\n");
////            odometryWriter.write(getAngle() + "\n");
////        } catch (IOException e) {
////            throw new RuntimeException("odometry file writer write failed: " + e.toString());
////        }
////        try {
////            RobotLog.d("odometryWriter.close");
////            odometryWriter.close();
////        } catch (IOException e) {
////            throw new RuntimeException("odometry file writer close failed: " + e.toString());
////        }
//        try {
//            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(context.openFileOutput("odometry positions.txt", Context.MODE_PRIVATE));
//
//            // write each configuration parameter as a string on its own line
//            outputStreamWriter.write(xBlue + "\n");
//            outputStreamWriter.write(yBlue + "\n");
//            outputStreamWriter.write(thetaDEG + "\n");
//            outputStreamWriter.write(getAngle() + "\n");
//
//            outputStreamWriter.close();
//        }
//        catch (IOException e) {
//            opMode.telemetry.addData("Exception", "Configuration file write failed: " + e.toString());
//        }
//
//    }

//    public void readPosition() {
//        try {
//            InputStream inputStream = context.openFileInput("odometry positions.txt");
//            if ( inputStream != null ) {
//                InputStreamReader inputStreamReader = new InputStreamReader(inputStream);
//                BufferedReader bufferedReader = new BufferedReader(inputStreamReader);
//
//                xBlue = Double.parseDouble(bufferedReader.readLine());
//                opMode.telemetry.addData("X:", xBlue);
//                yBlue = Double.parseDouble(bufferedReader.readLine());
//                opMode.telemetry.addData("Y:", yBlue);
//                opMode.telemetry.update();
//                RobotLog.d(String.format("odometry bodoo: %.2f, %.2f", xBlue, yBlue));
//                thetaDEG = Double.parseDouble(bufferedReader.readLine());
//                savedStartAngle = Double.parseDouble(bufferedReader.readLine());
//                thetaDEG = savedStartAngle;
//
//                inputStream.close();
//            }
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
//    }
}

package org.firstinspires.ftc.teamcode.SystemControl;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PID.PIDController;
import org.firstinspires.ftc.teamcode.PID.SPID;

/**
 * Created by Michael on 11/17/2017.
 */

/*
 * TODO: Tune PID!!!
 */
public class HardwareController {
    ////// Hardware Objects
    HardwareMap hardwareMap;
    Telemetry telemetry;
    // Motors
    private DcMotor motorDriveRightFront = null, motorDriveRightRear = null;    // Right side mecanum motors
    private DcMotor motorDriveLeftFront = null, motorDriveLeftRear = null;      // Left side mecanum motors
    private DcMotor motorDriveStrafe = null;    // Motor for strafing

    private DcMotor motorLift = null;   // Motor for the lift

    /* ------ OLD LIFT OBJECTS
    private DcMotor motorLiftInterior = null, motorLiftExterior = null;         // Lift motors
    private boolean useInteriorLiftMotor = false;   // Do we move the lift with the interior motor or the exterior?
    ------------ */

    // Distance Measurement Values
    private static final double driveEncoderTicksPerRotation = 1120;    // 1120 for the Neverest 40 ???
    private static final double driveWheelDiameterInches = 4;   // Diameter of the wheels

    // Servos
    private double leftOpenPos = 0.65, leftClosedPos = 0.10;
    private double rightOpenPos = 0.2, rightClosedPos = 0.75;
    private double rightGripPos = rightOpenPos, leftGripPos = leftOpenPos;
    private Servo servoGripLeft = null, servoGripRight = null;
    private double armUpPos = 1.0, armDownPos = 0.1;
    private Servo servoBallArm = null;
    // Waving
    private double waveStartTime = -1;
    private double waveDelay = 250;

    //Sensors
    private ColorSensor ballColorSensorLeft = null;    // Color sensor for balls
    private ColorSensor ballColorSensorRight = null;   // Color sensor for balls
    private BNO055IMU imu = null;  //Internal Gyro

    /*------ OLD LIFT SENSOR VALUES ------
    // Lift Limit Switch Sensors
    private DigitalChannel liftIntUpLs, liftIntDownLs;  // Interior lift limit switches (up, down)
    private DigitalChannel liftExtUpLs, liftExtDownLs;  // Exterior lift limit switches (up, down)
    // Lift position values
    private boolean liftIntUpState = false, liftIntDownState = false;   // Interior lift position states
    private boolean liftExtUpState = false, liftExtDownState = false;   // Exterior lift position states

    ------------*/


    private OpticalDistanceSensor leftOpticalDistance = null, rightOpticalDistance = null;   // Optical Distance Sensors on the left and right grippers
    // ODS values
    private double leftODSDetected = 0.0, rightODSDetected = 0.0;   // Left and right distance from the left and right ODSs

    // I2C MRI Range Sensor
    // Variables for reading over i2c
    private byte[] rangeCache;  // Cache for reading from the I2C Sensor
    private I2cAddr rangeAddress = new I2cAddr(0x14);  // Default I2C Address for MR range sensor
    private static final int rangeRegStart = 0x04;  // Register to start reading
    private static final int rangeReadLength = 2;   // Number of bytes to read;
    private I2cDevice range = null;    // Actual sensor device over I2C
    private I2cDeviceSynch rangeReader = null; // Reader for the I2C device
    // Sensor Values
    private double rangeODSValue = -1.0;
    private double rangeUltraSonicValue = -1.0;

    // GYRO and Turning
    private PIDController turnPIDController;
    private Orientation angles;
    private double zeroedHeading = 0.0;
    private double adjustedHeading = 0.0;
    private double heading = 0.0;
    private double roll = 0.0;
    private double pitch = 0.0;
    private Acceleration acceleration;

    // Encoders and Driving Distance
    private PIDController drivePIDController;
    private double rightFrontWheelDistance = 0, rightRearWheelDistance = 0;
    private double leftFrontWheelDistance = 0, leftRearWheelDistance = 0;
    private double avgLeftDistance = 0, avgRightDistance = 0;
    private double avgTotalDistance = 0;
    private double leftFrontEncZeroVal = 0.0, leftRearEncZeroVal = 0.0;
    private double rightFrontEncZeroVal = 0.0, rightRearEncZeroVal = 0.0;


    // Time from last turn (We need some latency because the motors don't stop right when the joystick is released)
    private double turnStartTime = -1;
    private double turnDuration = 0;
    private double turnWait = 15;   // Wait time for turn in ms

    // Drive Time Start Time
    private double driveTimeStart = -1.0;
    private double waitTimeStart = -1.0;

    // Speed modifiers
    private double turnMod = -0.75;
    private double speedMod = 0.75;
    private double omniXMod = -1;
    private double omniYMod = -1;
    private double omniStrafeMod = 0.75;
    private double mecanumXMod = -1;
    private double mecanumYMod = -1;

    // Lift speed modifier
    private double liftMod = -1;

    // Saved button states
    private boolean lastDriveAState = false;


    // ENUMS
    public enum InitError {Success, MotorInit, ServoInit, GyroInit, ColorInit, RangeInit, ODS, LimitSwitches};
    public enum ControlError {Success, Drive, Servo, Gyro, Lift, Color, Range, ODS, LimitSwitches};

    public InitError initErrorStatus = InitError.Success;
    public ControlError controlErrorStatus = ControlError.Success;


    public void initHardware(HardwareMap hm, Telemetry tel) {
        hardwareMap = hm;
        telemetry = tel;

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Initialize the motors
        try {
            motorDriveLeftFront = hardwareMap.get(DcMotor.class, "leftfront");
            motorDriveLeftRear = hardwareMap.get(DcMotor.class, "leftrear");
            motorDriveRightFront = hardwareMap.get(DcMotor.class, "rightfront");
            motorDriveRightRear = hardwareMap.get(DcMotor.class, "rightrear");
            motorDriveStrafe = hardwareMap.get(DcMotor.class, "strafe");
            // Set the motor directions
            motorDriveLeftFront.setDirection(DcMotor.Direction.FORWARD);
            motorDriveLeftRear.setDirection(DcMotor.Direction.FORWARD);
            motorDriveRightFront.setDirection(DcMotor.Direction.REVERSE);
            motorDriveRightRear.setDirection(DcMotor.Direction.REVERSE);
            motorDriveStrafe.setDirection(DcMotor.Direction.FORWARD);

            // Start the encoders
            motorDriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorDriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorDriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorDriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorDriveStrafe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Run Using Encoders
            motorDriveLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorDriveLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorDriveRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorDriveRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorDriveStrafe.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // lift motor
            motorLift = hardwareMap.get(DcMotor.class, "lift");

            /* ------ INIT FOR OLD LIFT
            motorLiftInterior = hardwareMap.get(DcMotor.class, "liftint");
            motorLiftExterior = hardwareMap.get(DcMotor.class, "liftext");
            ------------ */

        } catch (Exception ex) {
            initErrorStatus = InitError.MotorInit;
        }

        // Initialize the servos
        try {
            servoGripLeft = hardwareMap.get(Servo.class, "leftgrab");
            servoGripRight = hardwareMap.get(Servo.class, "rightgrab");

            servoBallArm = hardwareMap.get(Servo.class, "ballarm");

        } catch (Exception ex) {
            initErrorStatus = InitError.ServoInit;
        }

        // Setup the gyro
        try {
            // Set the parameters
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Initialize the gyro
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        } catch (Exception ex) {
            initErrorStatus = InitError.GyroInit;
        }

        // Setup the Color Sensors
        try {
            ballColorSensorLeft = hardwareMap.get(ColorSensor.class, "leftball");
            ballColorSensorRight = hardwareMap.get(ColorSensor.class, "rightball");
            ballColorSensorLeft.enableLed(true);
            ballColorSensorRight.enableLed(true);
        } catch (Exception ex) {
            initErrorStatus = InitError.ColorInit;
        }

        try {
            leftOpticalDistance = hardwareMap.get(OpticalDistanceSensor.class, "leftods");
            rightOpticalDistance = hardwareMap.get(OpticalDistanceSensor.class, "rightods");
        } catch (Exception ex) {
            initErrorStatus = InitError.ODS;
        }

        // Limit switches
        try {

            /* ------- OLD LIFT LIMIT SWITCHES ------
            liftIntUpLs = hardwareMap.get(DigitalChannel.class, "intup");
            liftIntDownLs = hardwareMap.get(DigitalChannel.class, "intdown");

            liftExtUpLs = hardwareMap.get(DigitalChannel.class, "extup");
            liftExtDownLs = hardwareMap.get(DigitalChannel.class, "extdown");
            ------------ */
        } catch (Exception ex) {
            initErrorStatus = InitError.LimitSwitches;
        }

        // Setup the Range Sensor on I2C
        try {
            //range = hardwareMap.i2cDevice.get("range");
            //rangeReader = new I2cDeviceSynchImpl(range, rangeAddress, false);
            //rangeReader.engage();
        } catch (Exception ex) {
            initErrorStatus = InitError.RangeInit;
        }

        // Set PID For Turning
        turnPIDController = new PIDController();
        SPID turnSPID = new SPID();
        turnSPID.iMax = 1.0;
        turnSPID.iMin = -1.0;
        turnSPID.pGain = 0.0;
        turnSPID.iGain = 0.0;
        turnSPID.dGain = 0.0;
        turnPIDController.setSPID(turnSPID);

        // Set PID For Driving
        drivePIDController = new PIDController();
        SPID driveSPID = new SPID();
        driveSPID.iMax = 1.0;
        driveSPID.iMin = -1.0;
        driveSPID.pGain = 0.0;
        driveSPID.iGain = 0.0;
        driveSPID.dGain = 0.0;
        drivePIDController.setSPID(driveSPID);


        telemetry.addData("Status", "Initialized, Error Code: " + initErrorStatus.toString());
        telemetry.update();
    }

    public double flipTurnMod() {
        double oldMod = turnMod;
        turnMod *= -1;
        return oldMod;  // Returns turnMod before it was changed so we can maybe change it back
    }

    public boolean waitIterative(double ms) {   // Simple function to wait for a given amount of time iteratively
        if (waitTimeStart < 0) waitTimeStart = System.currentTimeMillis();
        if (System.currentTimeMillis() - waitTimeStart >= ms) {
            waitTimeStart = -1;
            return false;
        }
        return true;    // Return false when done, do this in ALL functions like this
    }

    public void updateSensorsAndTelmetry() {
        waveRightGripper();
        updateEncoders();
        updateGYROValues();
        updateGripperODS();
        //updateRangeSensor();
        addErrorTelemetry();
        telemetry.addData("Right Gripper Servo Position", rightGripPos);
        telemetry.addData("Left Gripper Servo Position", leftGripPos);
        updateTelemetry();
    }
    private void addErrorTelemetry() {
        telemetry.addData("Init Status Code", initErrorStatus.toString());
        telemetry.addData("Control Status Code", controlErrorStatus.toString());
    }
    private void updateTelemetry() { telemetry.update();}


    ////// Control motors and servos
    // Mecanum drive
    public void MecanumDrive(double x, double y, double turn) {
        x *= mecanumXMod;
        y *= mecanumYMod;
        turn *= turnMod;
        double r = Math.hypot(x,y);
        double robotAngle = Math.atan2(y,x) - Math.PI / 4;

        final double v1 = r * Math.cos(robotAngle) + turn;
        final double v2 = r * Math.sin(robotAngle) - turn;
        final double v3 = r * Math.sin(robotAngle) + turn;
        final double v4 = r * Math.cos(robotAngle) - turn;

        try {
            motorDriveLeftFront.setPower(v1 * speedMod);
            motorDriveRightFront.setPower(v2 * speedMod);
            motorDriveLeftRear.setPower(v3 * speedMod);
            motorDriveRightRear.setPower(v4 * speedMod);
        } catch (Exception ex) {
            controlErrorStatus = ControlError.Drive;
        }
    }
    // OmniDrive
    public void OmniDrive(double x, double y, double turn) {
        x *= omniXMod;
        y *= omniYMod;
        turn *= turnMod;

        final double leftPower = y + turn;
        final double rightPower = y - turn;
        final double strafePower = x * omniStrafeMod;

        try {
            motorDriveLeftFront.setPower(leftPower * speedMod);
            motorDriveLeftRear.setPower(leftPower * speedMod);
            motorDriveRightFront.setPower(rightPower * speedMod);
            motorDriveRightRear.setPower(rightPower * speedMod);
            motorDriveStrafe.setPower(strafePower * speedMod);
        } catch (Exception ex) {
            controlErrorStatus = ControlError.Drive;
        }
    }
    // OmniDrive Tank
    public void OmniDriveTank(double left, double right, double strafe) {
        left *= omniYMod * speedMod;
        right *= omniYMod * speedMod;
        strafe *= omniStrafeMod;

        double leftPower = right;
        double rightPower = left;

        try {
            motorDriveLeftFront.setPower(leftPower);
            motorDriveLeftRear.setPower(leftPower);
            motorDriveRightFront.setPower(rightPower);
            motorDriveRightRear.setPower(rightPower);
            motorDriveStrafe.setPower(strafe);
        } catch (Exception ex) {
            controlErrorStatus = ControlError.Drive;
        }
    }

    // Function for driving forward distance
    public void driveForwardDistanceNoPIDIterative(double inches) {
        // This function drives until the distance (given in inches because we're in the US) is reached

    }

    // Function for driving with time
    public boolean driveTimeIterative(double ms, double forward, double strafe, double turn) {
        if (driveTimeStart < 0)  driveTimeStart = System.currentTimeMillis();
        double duration = System.currentTimeMillis() - driveTimeStart;
        if (duration >= ms)  {
            OmniDrive(0, 0, 0);
            driveTimeStart = -1;
            return false;    // Return false when the time is up
        }
        OmniDrive(strafe,forward,turn);
        return true;   // Return true when we need to wait longer
    }

    // CONTROL FOR NEW LIFT
    // Control the lift
    public void controlLift(double power) {
        try {
            motorLift.setPower(power);
        } catch (Exception ex) {
            controlErrorStatus = ControlError.Lift;
        }
    }
    /*  ------ CONTROL FOR OLD LIFT ------
    public void controlLift(double power) {
        try {
            if (useInteriorLiftMotor) {
                motorLiftExterior.setPower(0);
                motorLiftInterior.setPower(-power);
            } else {
                motorLiftInterior.setPower(0);
                motorLiftExterior.setPower(-power);
            }
        } catch (Exception ex) {
            controlErrorStatus = ControlError.Lift;
        }
    }
    public void setLiftMoveMotor(boolean interior) { useInteriorLiftMotor = interior; }
    public void controlLiftAutoSwitch(double power) {
        boolean down = power < 0;
        power *= liftMod;
        try {
            if (moveExtLift(down)) motorLiftExterior.setPower(power);
            else motorLiftExterior.setPower(0);

            if (moveIntLift(down)) motorLiftInterior.setPower(power);
            else motorLiftInterior.setPower(0);
        } catch (Exception ex) {
            controlErrorStatus = ControlError.Lift;
        }
    }
    public void controlLiftAutoSwitch_noIntSensors(double power) {
        boolean down = power < 0;
        power *= liftMod;
        try {
            if (moveExtLift(down)) {
                motorLiftExterior.setPower(power);
                motorLiftInterior.setPower(0);
            } else {
                motorLiftInterior.setPower(power);
                motorLiftExterior.setPower(0);
            }
        } catch (Exception ex) {
            controlErrorStatus = ControlError.Lift;
        }
    }
    ------------*/

    // Gripper Servos
    public void openCloseBlockGripper(boolean closed) {
        if (closed) {
            controlLeftBlockGripper(leftClosedPos);
            controlRightBlockGripper(rightClosedPos);
            //controlServo(servoGripLeft, leftClosedPos);
            //controlServo(servoGripRight, rightClosedPos);
        } else {
            controlLeftBlockGripper(leftOpenPos);
            controlRightBlockGripper(rightOpenPos);
            //controlServo(servoGripLeft, leftOpenPos);
            //controlServo(servoGripRight, rightOpenPos);
        }
    }
    public void variableControlBlockGripper(double amountMove) {
        amountMove *= 0.1;
        controlLeftBlockGripper(leftGripPos + amountMove);
        controlRightBlockGripper(rightGripPos - amountMove);
    }
    public void controlRightBlockGripper(double position) {
        rightGripPos = position;
        controlServo(servoGripRight, rightGripPos);
    }
    public void controlLeftBlockGripper(double position) {
        leftGripPos = position;
        controlServo(servoGripLeft, leftGripPos);
    }
    public void startRightWave(boolean wave) {
        if (wave && waveStartTime < 0) waveStartTime = System.currentTimeMillis();
    }
    // Gripper right Servo
    public void waveRightGripper() {
        if (waveStartTime < 0) return;
        double duration = System.currentTimeMillis() - waveStartTime;
        double c = rightClosedPos;
        if (duration >= 0 && duration < waveDelay * 0.5) c = rightClosedPos;
        if (duration >= waveDelay) c = rightOpenPos;
        if (duration >= waveDelay * 2) c = rightClosedPos;
        if (duration >= waveDelay * 3) c = rightOpenPos;
        if (duration >= waveDelay * 4) waveStartTime = -1;
        controlServo(servoGripRight, c);
    }
    // Arm Servo
    public void raiseLowerArm(boolean down) {
        if (down) {
            controlServo(servoBallArm, armDownPos);
        } else {
            controlServo(servoBallArm, armUpPos);
        }
    }

    // Control specified servo and catch any errors
    private void controlServo(Servo servo, double c) {
        try {
            servo.setPosition(c);
        } catch (Exception ex) {
            controlErrorStatus = ControlError.Servo;
        }
    }


    // Get and set gyro values
    private void updateGYROValues() {
        try {
            angles = imu.getAngularOrientation();
            heading = angles.firstAngle;
            roll = angles.secondAngle;
            pitch = angles.thirdAngle;

            // Calculate adjusted heading
            adjustedHeading = heading - zeroedHeading;

            // Acceleration
            acceleration = imu.getAcceleration();

            telemetry.addData("Heading", heading);
            telemetry.addData("Adjusted Heading", adjustedHeading);
        } catch (Exception ex) {
            controlErrorStatus = ControlError.Gyro;
        }
    }

    public double getTurnPID(double target) {
        double error = target - adjustedHeading;
        if (Math.abs(error) < 1) return 0;
        return turnPIDController.update(error, adjustedHeading);
    }

    public void setZeroedHeading() { zeroedHeading = heading; }

    // Color Sensors
    public String getLeftBallColor() { return getBallColor(ballColorSensorLeft); }
    public String getRightBallColor() { return getBallColor(ballColorSensorRight); }

    private float[] getColorSensorHSV(ColorSensor cs) {
        float hsvValues[] = {0F, 0F, 0F};
        int[] sensorRGB = getColorSensorRGB(cs);

        Color.RGBToHSV(sensorRGB[0], sensorRGB[1], sensorRGB[2], hsvValues);
        return hsvValues;
    }
    private int[] getColorSensorRGB(ColorSensor cs) {
        int rgbValues[] = {0,0,0};
        try {
            rgbValues = new int[]{cs.red(), cs.green(), cs.blue()};
        } catch (Exception ex) {
            controlErrorStatus = ControlError.Color;
        }
        return rgbValues;
    }
    private String getBallColor(ColorSensor cs) {
        float hue = getColorSensorHSV(cs)[0];
        int[] rgb = getColorSensorRGB(cs);

        if (rgb[0] > rgb[1] && rgb[0] > rgb[2]) {
            if (hue > 0 && hue < 10) {
                return "red";
            }
        }

        if (rgb[2] > rgb[0] && rgb[2] > rgb[1]) {
            if (hue > 200 && hue < 255) {
                return "blue";
            }
        }

        return "none";
    }

    // Range sensor control
    private void updateRangeSensor() {
        try {
            rangeCache = rangeReader.read(rangeRegStart, rangeReadLength);
            rangeUltraSonicValue = rangeCache[0] & 0xFF;
            rangeODSValue = rangeCache[1] & 0xFF;
        } catch (Exception ex) {
            controlErrorStatus = ControlError.Range;
        }
    }
    // Get the values from the sensor
    public double getRangeUltraSonicValue() { return rangeUltraSonicValue; }
    public double getRangeODSValue() { return rangeODSValue; }

    ////// Encoders and drive distance
    // Update distance
    private void updateEncoders() {
        double leftFrontTicks = motorDriveLeftFront.getCurrentPosition();
        double leftRearTicks = motorDriveLeftRear.getCurrentPosition();

        double rightFrontTicks = motorDriveRightFront.getCurrentPosition();
        double rightRearTicks = motorDriveRightRear.getCurrentPosition();


        //1120 Ticks Per Rotation with NeverRest 40 ???
        double leftFrontRotations = leftFrontTicks / driveEncoderTicksPerRotation;
        double leftRearRotations = leftRearTicks / driveEncoderTicksPerRotation;

        double rightFrontRotations = rightFrontTicks / driveEncoderTicksPerRotation;
        double rightRearRotations = rightRearTicks / driveEncoderTicksPerRotation;

        double wheelCircumferenceInches = 2 * Math.PI * driveWheelDiameterInches;

        telemetry.addData("Left Front Rotations", leftFrontRotations);

        leftFrontWheelDistance = leftFrontRotations * wheelCircumferenceInches;
        leftRearWheelDistance = leftRearRotations * wheelCircumferenceInches;

        rightFrontWheelDistance = rightFrontRotations * wheelCircumferenceInches;
        rightRearWheelDistance = rightRearRotations * wheelCircumferenceInches;

        avgLeftDistance = (leftFrontWheelDistance + leftRearWheelDistance) / 2;
        avgRightDistance = (rightFrontWheelDistance + rightRearWheelDistance) / 2;

        avgTotalDistance = (avgLeftDistance + avgRightDistance) / 2;
    }

    // Function to set the zeroed encoder values
    public void zeroDriveEncoders() {
        try {
            leftFrontEncZeroVal = motorDriveLeftFront.getCurrentPosition();
            leftRearEncZeroVal = motorDriveLeftRear.getCurrentPosition();
            rightFrontEncZeroVal = motorDriveRightFront.getCurrentPosition();
            rightRearEncZeroVal = motorDriveRightRear.getCurrentPosition();
        } catch (Exception ex) {
            controlErrorStatus = ControlError.Drive;
        }
    }

    // Get the distances
    public double getRightFrontWheelDistance() { return rightFrontWheelDistance; }
    public double getRightRearWheelDistance() { return rightRearWheelDistance; }
    public double getLeftFrontWheelDistance() { return leftFrontWheelDistance; }
    public double getLeftRearWheelDistance() { return leftRearWheelDistance; }

    public double getAvgLeftDistance() { return avgLeftDistance; }
    public double getAvgRightDistance() { return avgRightDistance; }
    public double getAvgTotalDistance() { return avgTotalDistance; }

    // Left and right ODS Control
    private void updateGripperODS() {
        try {
            leftODSDetected = leftOpticalDistance.getLightDetected();
            rightODSDetected = rightOpticalDistance.getLightDetected();
        } catch (Exception ex) {
            controlErrorStatus = ControlError.ODS;
        }
    }
    // Get the sensor values
    public double getLeftODSDetected() { return leftODSDetected; }
    public double getRightODSDetected() { return rightODSDetected; }

    /*  ------ SENSOR CONTROL FOR OLD DOUBLE LIFT
    //// Lift limit switches
    // Limit switch values as string
    private boolean moveExtLift(boolean down) {
        String position = getExtLiftPos();
        if (down && position != "down") return true;
        if (!down && position != "up") return true;
        return false;
    }
    private boolean moveIntLift(boolean down) {
        String position = getIntLiftPos();
        if (down && position != "down") return true;
        if (!down && position != "up") return true;
        return false;
    }
    private String getLiftPos(DigitalChannel upLs, DigitalChannel downLs) {
        try {
            if (upLs.getState() && !downLs.getState()) return "up";
            else if (!upLs.getState() && downLs.getState()) return "down";
        } catch (Exception ex) {
            controlErrorStatus = ControlError.LimitSwitches;
        }
        return "none";
    }
    // Get the lift positions
    public String getExtLiftPos() {
        return getLiftPos(liftExtUpLs, liftExtDownLs);
    }
    public String getIntLiftPos() {
        return getLiftPos(liftIntUpLs, liftIntDownLs);
    }
    */
}

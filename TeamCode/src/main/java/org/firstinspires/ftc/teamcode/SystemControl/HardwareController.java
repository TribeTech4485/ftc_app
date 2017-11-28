package org.firstinspires.ftc.teamcode.SystemControl;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.code.Attribute;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PID.PIDController;
import org.firstinspires.ftc.teamcode.PID.SPID;

/**
 * Created by Michael on 11/17/2017.
 */

public class HardwareController {
    ////// Hardware Objects
    HardwareMap hardwareMap;
    Telemetry telemetry;
    // Motors
    private DcMotor motorDriveRightFront = null, motorDriveRightRear = null;    // Right side mecanum motors
    private DcMotor motorDriveLeftFront = null, motorDriveLeftRear = null;      // Left side mecanum motors
    private DcMotor motorLift = null;   // Lift Motor
    // Servos
    private double leftOpenPos = 0.70, leftClosedPos = 0.10;
    private double rightOpenPos = 0.1, rightClosedPos = 0.9;
    private Servo servoGripLeft = null, servoGripRight = null;
    private double armUpPos = 0.25, armDownPos = 0.84;
    private Servo servoArm = null;

    //Sensors
    private ColorSensor ballColorSensorLeft;    // Color sensor for balls
    private ColorSensor ballColorSensorRight;   // Color sensor for balls
    private BNO055IMU imu;  //Internal Gyro

    // GYRO and Turning
    private PIDController pid;
    private Orientation angles;
    private double zeroedHeading = 0.0;
    private double adjustedHeading = 0.0;
    private double heading = 0.0;
    private double roll = 0.0;
    private double pitch = 0.0;

    // Time from last turn (We need some latency because the motors don't stop right when the joystick is released)
    private double turnStartTime = -1;
    private double turnDuration = 0;
    private double turnWait = 15;   // Wait time for turn in ms

    // Speed modifiers
    private double turnMod = 1.0;
    private double speedMod = 1.0;

    // Saved button states
    private boolean lastDriveAState = false;


    // ENUMS
    public enum InitError {Success, MotorInit, ServoInit, GyroInit};
    public enum ControlError {Success, Drive, Servo, Gyro, Lift};

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
            // Set the motor directions
            motorDriveLeftFront.setDirection(DcMotor.Direction.FORWARD);
            motorDriveLeftRear.setDirection(DcMotor.Direction.FORWARD);
            motorDriveRightFront.setDirection(DcMotor.Direction.REVERSE);
            motorDriveRightRear.setDirection(DcMotor.Direction.REVERSE);

            // lift motor
            motorLift = hardwareMap.get(DcMotor.class, "lift");
        } catch (Exception ex) {
            initErrorStatus = InitError.MotorInit;
        }

        // Initialize the servos
        try {
            servoGripLeft = hardwareMap.get(Servo.class, "leftgrab");
            servoGripRight = hardwareMap.get(Servo.class, "rightgrab");
            servoArm = hardwareMap.get(Servo.class, "ballarm");
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

        // Set PID
        pid = new PIDController();
        SPID spid = new SPID();
        spid.iMax = 1.0;
        spid.iMin = -1.0;
        spid.pGain = 0.0475;
        spid.iGain = 0.0;
        spid.dGain = 002375;
        pid.setSPID(spid);

        telemetry.addData("Status", "Initialized, Error Code: " + initErrorStatus.toString());
        telemetry.update();
    }

    public void updateSensorsAndTelmetry() {
        updateGYROValues();
        addErrorTelemetry();
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
    public void controlLift(double power) {
        try {
            motorLift.setPower(power);
        } catch (Exception ex) {
            controlErrorStatus = ControlError.Lift;
        }
    }

    // Gripper Servos
    public void openCloseBlockGripper(boolean closed) {
        if (closed) {
            controlServo(servoGripLeft, leftOpenPos);
            controlServo(servoGripRight, rightOpenPos);
        } else {
            controlServo(servoGripLeft, leftClosedPos);
            controlServo(servoGripRight, rightClosedPos);
        }
    }
    // Arm Servo
    public void raiseLowerArm(boolean down) {
        if (down) {
            controlServo(servoArm, armDownPos);
        } else {
            controlServo(servoArm, armUpPos);
        }
    }

    // Control specified servo and catch any errors
    private void controlServo(Servo servo, double c) {
        try {
            servo.setPosition(c);
        } catch (Exception ex) {
            controlErrorStatus = ControlError.Gyro;
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

            telemetry.addData("Heading", heading);
            telemetry.addData("Adjusted Heading", adjustedHeading);
        } catch (Exception ex) {
            controlErrorStatus = ControlError.Gyro;
        }
    }

    public double getTurnPID(double target) {
        double error = target - adjustedHeading;
        if (Math.abs(error) < 1) return 0;
        return pid.update(error, adjustedHeading);
    }

    public void setZeroedHeading() { zeroedHeading = heading; }
}


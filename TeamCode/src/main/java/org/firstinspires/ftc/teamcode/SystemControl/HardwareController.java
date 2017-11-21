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
    // Servos
    private Servo servoGripLeft = null, servoGripRight = null;

    //Sensors
    private ColorSensor ballColorSensor;    // Color sensor for balls
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
    public enum UpdateError {Success};

    public InitError initErrorStatus;
    public UpdateError updateErrorStatus;


    public void initHardware(HardwareMap hm, Telemetry tel) {
        hardwareMap = hm;
        telemetry = tel;

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Initialize the drive motors
        try {
            motorDriveLeftFront = hardwareMap.get(DcMotor.class, "driveLeftFront");
            motorDriveLeftRear = hardwareMap.get(DcMotor.class, "driveLeftRear");
            motorDriveRightFront = hardwareMap.get(DcMotor.class, "driveRightFront");
            motorDriveRightRear = hardwareMap.get(DcMotor.class, "driveRightRear");
            // Set the motor directions
            motorDriveLeftFront.setDirection(DcMotor.Direction.FORWARD);
            motorDriveLeftRear.setDirection(DcMotor.Direction.FORWARD);
            motorDriveRightFront.setDirection(DcMotor.Direction.REVERSE);
            motorDriveRightRear.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception ex) {
            // TODO: Error flag or something
            initErrorStatus = InitError.MotorInit;
        }

        // Initialize the servos
        try {
            servoGripLeft = hardwareMap.get(Servo.class, "leftGrab");
            servoGripRight = hardwareMap.get(Servo.class, "rightGrab");
        } catch (Exception ex) {
            // TODO: Error flag or something
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
            // TODO: Error flag or something
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

        telemetry.addData("Status", ("Initialized, Error Code: " + initErrorStatus.toString()));
        telemetry.update();
    }
}

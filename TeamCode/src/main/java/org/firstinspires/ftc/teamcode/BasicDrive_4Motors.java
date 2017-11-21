package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PID.PIDController;
import org.firstinspires.ftc.teamcode.PID.SPID;

/**
 * Created by Michael on 9/16/2017.
 */

@TeleOp(name="BasicDrive4Motors")
public class BasicDrive_4Motors extends OpMode
{
    // OpMode Members
    private ElapsedTime runTime = new ElapsedTime();
    private DcMotor motorDriveRightFront = null, motorDriveRightBack = null;
    private DcMotor motorDriveLeftFront = null, motorDriveLeftBack = null;

    // Color Sensors
    private ColorSensor ballColorSensor;

    // The Internal GYRO
    private BNO055IMU imu;
    // PID For turning
    private PIDController pid;

    private Orientation angles;
    private double zeroedHeading = 0.0;
    private double adjustedHeading = 0.0;
    private double heading = 0.0;
    private double roll = 0.0;
    private double pitch = 0.0;

    // Time for last turn (because we need a delay before we correct fo turn)
    private double turnStartTime = -1;
    private double turnDuration = 0;
    private double turnWait = 15;   // Wait time for turn in ms

    private Servo leftGripServo, rightGripServo;

    // Speed modifiers
    private double turnMod = 1.0;
    private double speedMod = 1.0;

    // Saved button states
    private boolean lastDriveAState = false;

    // Initialize the robot when the init button is hit
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        // Initialize the hardware variables
        try {
            motorDriveRightFront = hardwareMap.get(DcMotor.class, "driveRightFront");
            motorDriveRightBack = hardwareMap.get(DcMotor.class, "driveRightBack");
            motorDriveLeftFront = hardwareMap.get(DcMotor.class, "driveLeftFront");
            motorDriveLeftBack = hardwareMap.get(DcMotor.class, "driveLeftBack");
            ballColorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        } catch (Exception ex) {

        }

        // Servos
        try {
            leftGripServo = hardwareMap.get(Servo.class, "leftGrab");
            rightGripServo = hardwareMap.get(Servo.class, "rightGrab");
        } catch (Exception ex) {

        }

        // GYRO
        // Set the parameters

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        try {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        } catch (Exception ex) {

        }

        pid = new PIDController();
        SPID spid = new SPID();
        spid.iMax = 1.0;
        spid.iMin = -1.0;
        spid.pGain = 0.0475;//0.054;//0.048;
        spid.iGain = 0.0;
        spid.dGain = 0.02375;
        pid.setSPID(spid);

        // Set the motor spin directions
        try {
            motorDriveRightFront.setDirection(DcMotor.Direction.REVERSE);
            motorDriveRightBack.setDirection(DcMotor.Direction.REVERSE);
            motorDriveLeftFront.setDirection(DcMotor.Direction.FORWARD);
            motorDriveLeftBack.setDirection(DcMotor.Direction.FORWARD);
        } catch (Exception ex) {

        }

        // Update the status on the driver station
        telemetry.addData("Status", "Initialized");
    }

    // Loop to run after init but before play
    @Override
    public void init_loop() {

    }

    // Run once after the play button is pressed
    @Override
    public void start() { runTime.reset(); }

    // Loop to run after play is hit and before stop
    @Override
    public void loop() {
        //double leftDrive;
        //double rightDrive;

        //leftDrive = -gamepad1.left_stick_y;
        //rightDrive = -gamepad1.right_stick_y;

        /*
        motorDriveLeftFront.setPower(leftDrive);
        motorDriveLeftBack.setPower(leftDrive);
        motorDriveRightFront.setPower(rightDrive);
        motorDriveRightBack.setPower(rightDrive);
        */
        updateGYROVals();
        getColor(ballColorSensor);

        if (turnStartTime > -1) {
            turnDuration = System.currentTimeMillis() - turnStartTime;
        }
        //TODO: delete all code


        double turnAmount = 0.0;
        if (Math.abs(gamepad1.right_stick_x) <= 0.01) {
            if (turnStartTime < 0) turnStartTime = System.currentTimeMillis();
            if (turnDuration >= turnWait) turnAmount = getTurnPID(0);
            //turnAmount = turnToAngleGetTurn(0);
            //turnAmount = getTurnPID(0);
        }
        else {
            turnAmount = gamepad1.right_stick_x;
            setZeroedHeading();
            turnStartTime = -1;
        }
        MecanumDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, turnAmount);

        boolean driveAState = gamepad1.a;
        if (driveAState != lastDriveAState) {
            lastDriveAState = driveAState;
            openCloseBlockGripper(driveAState);
        }

        telemetry.addData("Status", "Run Time: " + runTime.toString());
        telemetry.addData("Heading", heading);
        telemetry.addData("Roll", roll);
        telemetry.addData("Pitch", pitch);
        telemetry.update();
    }

    // Runs once after the stop button is hit
    @Override
    public void stop() {
        motorDriveLeftFront.setPower(0);
        motorDriveLeftBack.setPower(0);
        motorDriveRightFront.setPower(0);
        motorDriveRightBack.setPower(0);
    }

    private void MecanumDrive(double leftX, double leftY, double rightX) {
        rightX *= turnMod;
        double r = Math.hypot(leftX, leftY);
        double robotAngle = Math.atan2(leftY, leftX) - Math.PI / 4;

        telemetry.addData("Motors", "leftX (%.2f), leftY (%.2f), rightX (%.2f)", leftX, leftY, rightX);

        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        motorDriveLeftFront.setPower(v1 * speedMod);
        motorDriveRightFront.setPower(v2 * speedMod);
        motorDriveLeftBack.setPower(v3 * speedMod);
        motorDriveRightBack.setPower(v4 * speedMod);
    }

    private void openCloseBlockGripper(boolean closed) {
        leftGripServo.setPosition(0.5);
        rightGripServo.setPosition(0.5);
    }

    private double turnToAngleGetTurn(double target) {
        if (Math.abs(adjustedHeading) > target - 1 && Math.abs(adjustedHeading) < target + 1) return 0.0;
        if (adjustedHeading > target) {
            return -0.2;
        } else if (adjustedHeading < target) {
            return 0.2;
        }
        return 0.0;
    }

    private double getTurnPID(double target) {
        double error = target - adjustedHeading;
        if (Math.abs(error) < 1) return 0;
        return pid.update(error, adjustedHeading);
    }

    private void updateGYROVals() {
        try {
            angles = imu.getAngularOrientation();
            heading = angles.firstAngle;
            roll = angles.secondAngle;
            pitch = angles.thirdAngle;

            // Calculate adjusted heading
            adjustedHeading = heading - zeroedHeading;

            telemetry.addData("Heading", heading);
            telemetry.addData("Adjusted Heading", adjustedHeading);
            //telemetry.addData("Roll", roll);
            //telemetry.addData("Pitch", pitch);
        } catch (Exception ex) {

        }
    }

    private void setZeroedHeading() { zeroedHeading = heading; }

    private String getColor(ColorSensor cs) {
        int red = cs.red();
        int green = cs.green();
        int blue = cs.green();
        int alpha = cs.alpha();

        telemetry.addData("Red", red);
        telemetry.addData("Green", green);
        telemetry.addData("Blue", blue);
        telemetry.addData("Alpha", alpha);
        telemetry.update();

        return "";
    }

}

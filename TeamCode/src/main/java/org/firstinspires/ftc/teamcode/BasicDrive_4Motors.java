package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.abs;

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
        motorDriveRightFront = hardwareMap.get(DcMotor.class, "driveRightFront");
        motorDriveRightBack = hardwareMap.get(DcMotor.class, "driveRightBack");
        motorDriveLeftFront = hardwareMap.get(DcMotor.class, "driveLeftFront");
        motorDriveLeftBack = hardwareMap.get(DcMotor.class, "driveLeftBack");

        leftGripServo = hardwareMap.get(Servo.class, "leftGrab");
        rightGripServo = hardwareMap.get(Servo.class, "rightGrab");


        // Set the motor spin directions
        motorDriveRightFront.setDirection(DcMotor.Direction.REVERSE);
        motorDriveRightBack.setDirection(DcMotor.Direction.REVERSE);
        motorDriveLeftFront.setDirection(DcMotor.Direction.FORWARD);
        motorDriveLeftBack.setDirection(DcMotor.Direction.FORWARD
        );

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

        MecanumDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

        boolean driveAState = gamepad1.a;
        if (driveAState != lastDriveAState) {
            lastDriveAState = driveAState;
            openCloseBlockGripper(driveAState);
        }

        telemetry.addData("Status", "Run Time: " + runTime.toString());
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

}

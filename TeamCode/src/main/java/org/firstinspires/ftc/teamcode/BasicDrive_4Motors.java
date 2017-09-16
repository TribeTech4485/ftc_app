package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

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


    // Initialize the robot when the init button is hit
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        // Initialize the hardware variables
        motorDriveRightFront = hardwareMap.get(DcMotor.class, "driveRightFront");
        motorDriveRightBack = hardwareMap.get(DcMotor.class, "driveRightBack");
        motorDriveLeftFront = hardwareMap.get(DcMotor.class, "driveLeftFront");
        motorDriveLeftBack = hardwareMap.get(DcMotor.class, "driveLeftBack");

        // Set the motor spin directions
        motorDriveRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorDriveRightBack.setDirection(DcMotor.Direction.FORWARD);
        motorDriveLeftFront.setDirection(DcMotor.Direction.FORWARD);
        motorDriveLeftBack.setDirection(DcMotor.Direction.FORWARD);

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
        double leftDrive;
        double rightDrive;

        leftDrive = -gamepad1.left_stick_y;
        rightDrive = -gamepad1.right_stick_y;

        motorDriveLeftFront.setPower(leftDrive);
        motorDriveLeftBack.setPower(leftDrive);
        motorDriveRightFront.setPower(rightDrive);
        motorDriveRightBack.setPower(rightDrive);

        telemetry.addData("Status", "Run Time: " + runTime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftDrive, rightDrive);
    }

    // Runs once after the stop button is hit
    @Override
    public void stop() {
        motorDriveLeftFront.setPower(0);
        motorDriveLeftBack.setPower(0);
        motorDriveRightFront.setPower(0);
        motorDriveRightBack.setPower(0);
    }

}

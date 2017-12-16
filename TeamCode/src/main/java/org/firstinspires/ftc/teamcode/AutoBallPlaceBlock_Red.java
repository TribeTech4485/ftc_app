package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SystemControl.HardwareController;

/**
 * Created by Michael on 12/5/2017.
 */

//@Autonomous(name="AutoBallPlaceBlock_Red", group="Red")
public class AutoBallPlaceBlock_Red extends LinearOpMode {

    HardwareController hwcon;
    String targetBallColor = "red";

    void delay(double ms) {
        double startWait = System.currentTimeMillis();
        while(System.currentTimeMillis() - startWait < ms);
    }

    @Override
    public void runOpMode() {
        hwcon = new HardwareController();
        hwcon.initHardware(hardwareMap, telemetry);

        hwcon.updateSensorsAndTelmetry();

        //// Pick up block
        hwcon.openCloseBlockGripper(true);  // Close gripper
        delay(1500); // wait while gripper closes
        hwcon.controlLift(1.0); // Start moving
        delay(750); // Wait while lifter rises
        hwcon.controlLift(0.0); // Stop moving


        // Put the arm down
        hwcon.raiseLowerArm(true);
        delay(500);

        // Update the sensors again
        hwcon.updateSensorsAndTelmetry();

        String leftBallColor = "none";
        String rightBallColor = "none";
        // Get the color or the balls
        leftBallColor = hwcon.getLeftBallColor();   // Left ball
        rightBallColor = hwcon.getRightBallColor(); // Right ball

        telemetry.addData("LeftBall", leftBallColor);
        telemetry.addData("RightBall", rightBallColor);

        double moveDirection = 0;   // +1 right, -1 left, 0 none
        // Check the colors
        if (leftBallColor == "none" && rightBallColor != "none" && rightBallColor != targetBallColor) moveDirection = -1;
        if (leftBallColor != "none" && leftBallColor != targetBallColor && rightBallColor == "none") moveDirection = 1;
        if (leftBallColor == targetBallColor && rightBallColor == targetBallColor) moveDirection = 0;
        if (leftBallColor == targetBallColor && rightBallColor != targetBallColor) moveDirection = -1;
        if (leftBallColor != targetBallColor && rightBallColor == targetBallColor) moveDirection = 1;

        telemetry.addData("Direction", moveDirection);
        telemetry.update();

        // turn the direction for x ms
        double turnStart = System.currentTimeMillis();
        while ((System.currentTimeMillis() - turnStart) < 400) {
            hwcon.MecanumDrive(0,0,moveDirection);
        }

        hwcon.MecanumDrive(0,0,0);  // Stop moving
        hwcon.raiseLowerArm(false); // Raise the arm
        delay(400);

        turnStart = System.currentTimeMillis();
        while ((System.currentTimeMillis() - turnStart) < 400) {
            hwcon.MecanumDrive(0,0,-moveDirection);
        }

        hwcon.MecanumDrive(0,0,0);
        delay(1500);
    }

}

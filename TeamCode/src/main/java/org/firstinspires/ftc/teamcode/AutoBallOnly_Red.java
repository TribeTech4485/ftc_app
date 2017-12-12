package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.teamcode.SystemControl.HardwareController;

/**
 * Created by Michael on 12/5/2017.
 */

@Autonomous(name="AutoBallOnly_Red", group="Red")
public class AutoBallOnly_Red extends LinearOpMode {

    HardwareController hwcon;
    String targetBallColor = "red";


    @Override
    public void runOpMode() {
        hwcon = new HardwareController();
        hwcon.initHardware(hardwareMap, telemetry);

        hwcon.updateSensorsAndTelmetry();

        // Put the arm down
        hwcon.raiseLowerArm(true);

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
        {   // 400ms delay for the arm to come up
            double startWait = System.currentTimeMillis();
            while (System.currentTimeMillis() - startWait < 400);
        }

        turnStart = System.currentTimeMillis();
        while ((System.currentTimeMillis() - turnStart) < 400) {
            hwcon.MecanumDrive(0,0,-moveDirection);
        }

        hwcon.MecanumDrive(0,0,0);
    }

}

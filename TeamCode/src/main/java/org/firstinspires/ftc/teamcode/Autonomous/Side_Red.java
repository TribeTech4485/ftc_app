package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SystemControl.HardwareController;

/**
 * Created by Michael on 12/13/2017.
 */
@Autonomous (name="Side_Red", group="Red")
public class Side_Red extends LinearOpMode {
    HardwareController hwcon = new HardwareController();

    String teamColor = "red";

    @Override
    public void runOpMode() {
        hwcon.initHardware(hardwareMap, telemetry);
        //// Pick up block
        hwcon.openCloseBlockGripper(true);  // Close gripper
        while (hwcon.waitIterative(1500) && !isStopRequested());
        hwcon.controlLift(1.0); // Start moving
        while (hwcon.waitIterative(750) && !isStopRequested());
        hwcon.controlLift(0.0); // Stop moving
        hitBall();
        while (hwcon.waitIterative(500) && !isStopRequested());
        while (hwcon.driveTimeIterative(1200, -1.0, 0, 0) && !isStopRequested());
        while (hwcon.driveTimeIterative(1000, 0, 0, 1.0) && !isStopRequested());
        while (hwcon.driveTimeIterative(1900, -1.0, 0, 0) && !isStopRequested());
        hwcon.openCloseBlockGripper(false);
        while (hwcon.waitIterative(250) && !isStopRequested());
        while (hwcon.driveTimeIterative(350, 0,0, -1.0));
        while (hwcon.driveTimeIterative(-400, 1.0, 0, 0) && !isStopRequested());
        while (hwcon.driveTimeIterative(100, 1.0, 0, 0) && !isStopRequested());
        hwcon.OmniDrive(0,0,0);
    }

    private void hitBall() {
        hwcon.updateSensorsAndTelmetry();

        // Put the arm down
        hwcon.raiseLowerArm(true);
        {// Wait
            double startWait = System.currentTimeMillis();
            while(System.currentTimeMillis() - startWait < 1000);
        }

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
        if (leftBallColor == "none" && rightBallColor != "none" && rightBallColor != teamColor) moveDirection = 1;
        if (leftBallColor != "none" && leftBallColor != teamColor && rightBallColor == "none") moveDirection = -1;
        if (leftBallColor == teamColor && rightBallColor == teamColor) moveDirection = 0;
        if (leftBallColor == teamColor && rightBallColor != teamColor) moveDirection = 1;
        if (leftBallColor != teamColor && rightBallColor == teamColor) moveDirection = -1;

        telemetry.addData("Direction", moveDirection);
        telemetry.update();

        // turn the direction for x ms
        while (!hwcon.driveTimeIterative(400, 0, 0, moveDirection));

        hwcon.OmniDrive(0,0,0);  // Stop moving
        hwcon.raiseLowerArm(false); // Raise the arm
        {   // 400ms delay for the arm to come up
            double startWait = System.currentTimeMillis();
            while (System.currentTimeMillis() - startWait < 400);
        }

        while(!hwcon.driveTimeIterative(400, 0, 0, -moveDirection));

        hwcon.OmniDrive(0,0,0);
    }
}

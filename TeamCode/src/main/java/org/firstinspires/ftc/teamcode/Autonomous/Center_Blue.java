package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SystemControl.HardwareController;

/**
 * Created by Michael on 12/14/2017.
 */

@Autonomous (name="Center_Blue",group="Blue")
public class Center_Blue extends LinearOpMode {

    HardwareController hwcon = new HardwareController();
    String teamColor = "blue";

    @Override
    public void runOpMode() {

        hwcon.initHardware(hardwareMap, telemetry);
        //hwcon.flipTurnMod();    // this is blue auto so we have to turn the opposite direction every time
                                // this function will flip the turn modifier so that the robot turns the opposite direction
                                // !! run this again at the end of the program !!   - Note it might not matter if we change it back because
                                //                                                      HardwareController is a class and is re-instanced in every auto/teleOp mode

        waitForStart();

        //// Pick up block
        hwcon.openCloseBlockGripper(true);  // Close gripper
        while (hwcon.waitIterative(1500) && !isStopRequested());    // wait for the gripper to close
        hwcon.controlLift(1.0); // Start moving
        while (hwcon.waitIterative(750) && !isStopRequested());     // move the lift up for 750 ms
        hwcon.controlLift(0.0); // Stop moving

        hitBall(); // Hit other teams ball off


        hwcon.flipTurnMod();
        while (hwcon.waitIterative(500) && !isStopRequested()); // Wait for a little bit
        while (hwcon.driveTimeIterative(1200, -1.0, 0, 0) && !isStopRequested());   // Move off the balance pad
        while (hwcon.driveTimeIterative(1000, 0, 0, 1.0) && !isStopRequested());    // Turn facing the wall

        while (hwcon.driveTimeIterative(1500, -1.0, 0, 0) && !isStopRequested());   // Move forward past the balance pad
        while (hwcon.driveTimeIterative(1000, 0, 0, 1.0) && !isStopRequested());    // Move facing the goal

        // Hopefully put block in the goal
        while (hwcon.driveTimeIterative(1900, -1.0, 0, 0) && !isStopRequested());   // move forward to the goal
        hwcon.openCloseBlockGripper(false);     // Open the gripper
        while (hwcon.waitIterative(250) && !isStopRequested()); // Wait for the gripper to open
        while (hwcon.driveTimeIterative(350, 0,0, -1.0));       // Angle the block
        while (hwcon.driveTimeIterative(-400, 1.0, 0, 0) && !isStopRequested());    // move forward to push the block in
        while (hwcon.driveTimeIterative(100, 1.0, 0, 0) && !isStopRequested());     // back away from the block but stay in the scoring zone
        hwcon.OmniDrive(0,0,0); // stop moving

        hwcon.flipTurnMod();
    }

    private void hitBall() {
        hwcon.updateSensorsAndTelmetry();

        // Put the arm down
        //hwcon.raiseLowerSensorArm(true);
        while(hwcon.waitIterative(1500) && !isStopRequested());
        hwcon.raiseLowerArm(true);
        while(hwcon.waitIterative(1000) && !isStopRequested());

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
        // Put the arm down
        hwcon.raiseLowerArm(false);
        while(hwcon.waitIterative(1000) && !isStopRequested());
        //hwcon.raiseLowerSensorArm(false);
        while(hwcon.waitIterative(1500) && !isStopRequested());

        while(!hwcon.driveTimeIterative(400, 0, 0, -moveDirection));

        hwcon.OmniDrive(0,0,0);
    }
}

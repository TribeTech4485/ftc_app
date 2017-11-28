package org.firstinspires.ftc.teamcode.SystemControl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.SystemControl.HardwareController;

/**
 * Created by Michael on 11/21/2017.
 */

@TeleOp(name="DriveTele")
public class Drive4Motors extends OpMode {

    private HardwareController hwcon;
    private Gamepad drivePad, controlPad;



    @Override
    public void init() {
        drivePad = gamepad1;
        controlPad = gamepad1;
        hwcon = new HardwareController();
        hwcon.initHardware(hardwareMap, telemetry);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    private boolean gripperClosed = false;
    private boolean armDown = false;
    private double turnStartTime = -1;
    private double turnDuration = -1;
    private double turnWait = 500;
    private boolean yawZeroedSinceLastTurn = true;
    @Override
    public void loop() {

        if (controlPad.a) gripperClosed = true;
        else if (controlPad.b) gripperClosed = false;
        hwcon.openCloseBlockGripper(gripperClosed);

        if (controlPad.left_bumper) armDown = false;
        else if (controlPad.right_bumper) armDown = true;
        hwcon.raiseLowerArm(armDown);

        hwcon.controlLift(controlPad.right_trigger - controlPad.left_trigger);

        double turnAmount = 0.0;
        /*
        // If we are turning with the drive controller
        if (drivePad.right_stick_x > 0.01) {
            turnAmount = drivePad.right_stick_x;
            turnStartTime = System.currentTimeMillis();
            turnDuration = -1;
            yawZeroedSinceLastTurn = false;
        } else {
            // If we are not turning with the drive controller
            // Check how long it has been
            if (turnStartTime >= 0) turnDuration = System.currentTimeMillis() - turnStartTime;
            if (turnDuration >= turnWait) { // If the delay is over
                if (!yawZeroedSinceLastTurn) {  // If the yaw hasn't been zeroed since the last time we turned
                    //hwcon.setZeroedHeading();   // Zero the yaw
                    yawZeroedSinceLastTurn = true;  // Don't zero the yaw until next turn
                }
                //turnAmount = hwcon.getTurnPID(0);   // Turn to 0 degrees (say in the same orientation)
            }
        }*/
        hwcon.MecanumDrive(drivePad.left_stick_x, drivePad.left_stick_y, turnAmount);

        /* TODO: NullPointerException Somewhere in loop()
        *  TODO: Check hwcon for errors in function calls
         */
        // TODO: This is likely the issue
        hwcon.updateSensorsAndTelmetry();
    }

    @Override
    public void stop() {
        hwcon.MecanumDrive(0,0,0);
    }
}

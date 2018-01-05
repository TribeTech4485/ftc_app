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
    private Gamepad lastControlPad;


    @Override
    public void init() {
        drivePad = gamepad1;
        controlPad = gamepad2;
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

    private double lastLoopTime = -1;
    @Override
    public void loop() {

        double loopStart = System.currentTimeMillis();

        // move controls around if requested
        if (drivePad.start) {
            // Move controls to drive pad
            if (controlPad != drivePad) {
                lastControlPad = controlPad;
                controlPad = drivePad;
            }
        } else if (drivePad.back) {
            // Move controls back to control pad
            controlPad = lastControlPad;
        }

        //// Control the servos
        // Control the gripper with the control pad with a and b
        if (controlPad.a || controlPad.b) {
            if (controlPad.a) gripperClosed = true;
            else if (controlPad.b) gripperClosed = false;
            hwcon.openCloseBlockGripper(gripperClosed);
        } else if (controlPad.x) {
            // Control the block gripper with the joystick if x is pushed on the control pad
            hwcon.variableControlBlockGripper(controlPad.left_stick_y);
        }

        // Control the arm
        // TODO: Remove this, it isn't really necessary.
        //if (controlPad.left_bumper) armDown = false;
        //else if (controlPad.right_bumper) armDown = true;
        //hwcon.raiseLowerArm(armDown);

        // A friendly wave at the push of a button
        hwcon.startRightWave(drivePad.dpad_right);


        //// Control the lift
        hwcon.controlLift(controlPad.right_trigger - controlPad.left_trigger);

        /* ------ OLD LIFT CONTROL
        // New lift control
        hwcon.setLiftMoveMotor(controlPad.y);
        hwcon.controlLift(controlPad.right_trigger - controlPad.left_trigger);
        if (controlPad.dpad_down) {
            //hwcon.setLiftMoveMotor(controlPad.y);
            //  hwcon.controlLift(controlPad.right_trigger - controlPad.left_trigger);
        } else {
            //hwcon.controlLiftAutoSwitch_noIntSensors(controlPad.right_trigger - controlPad.left_trigger);
        }
        ------------*/

        // Switch between the interior and exterior lift
        // TODO: Deprecate this and replace it with auto switching.
        //hwcon.setLiftMoveMotor(drivePad.y);

        // Both the drive and control pad can move the lift
        // If there is drive pad input use that
        /*
        double controlLiftControlPad = controlPad.right_trigger - controlPad.left_trigger;
        double controlLiftDrivePad = drivePad.right_trigger - drivePad.left_trigger;
        if (controlLiftDrivePad != 0) hwcon.controlLift(controlLiftDrivePad);
        else if (controlLiftControlPad != 0) hwcon.controlLift(controlLiftControlPad);
        else hwcon.controlLift(0);
        */
        //hwcon.controlLift(drivePad.right_trigger - drivePad.left_trigger);


        //// Drive the drive base
        // Check if the controls are on the drive pad and if so, check if the x button is pushed
        double yAmount = 0.0;
        if (controlPad != drivePad || !drivePad.x) { // If the joystick isn't being used to control the servos, set the drive variable
            yAmount = drivePad.left_stick_y;
        }
        hwcon.OmniDriveTank(yAmount, drivePad.right_stick_y, (drivePad.left_stick_x));  // Tank drive using Omni Wheels

        // Loop Time Telemetry
        // TODO: Fix lag issues, 50 to 110 ms loop times
        telemetry.addData("Last Loop Time", lastLoopTime);

        // Optical Distance Sensors Telemetry
        telemetry.addData("Right ODS", hwcon.getRightODSDetected());
        telemetry.addData("Left ODS", hwcon.getLeftODSDetected());

        // Encoder Distance Telemetry
        telemetry.addData("Right Front Distance", hwcon.getRightFrontWheelDistance());
        telemetry.addData("Right Rear Distance", hwcon.getRightRearWheelDistance());
        telemetry.addData("Left Front Distance", hwcon.getLeftFrontWheelDistance());
        telemetry.addData("Left Rear Distance", hwcon.getLeftRearWheelDistance());

        telemetry.addData("Total Distace", hwcon.getAvgTotalDistance());
        telemetry.addData("AVG Left Distance", hwcon.getAvgLeftDistance());
        telemetry.addData("AVG Right Distance", hwcon.getAvgRightDistance());

        // Update the sensors and telemetry in the hardware controller
        hwcon.updateSensorsAndTelmetry();

        lastLoopTime = System.currentTimeMillis() - loopStart;  // Calculate the loop time
    }

    @Override
    public void stop() {
        hwcon.OmniDriveTank(0,0,0); // Stop the drive motors
    }
}

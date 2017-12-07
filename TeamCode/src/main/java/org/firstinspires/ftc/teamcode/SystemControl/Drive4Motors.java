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
    private double turnStartTime = -1;
    private double turnDuration = -1;
    private double turnWait = 80;
    private boolean yawZeroedSinceLastTurn = true;


    private double lastLoopTime = -1;
    @Override
    public void loop() {

        double loopStart = System.currentTimeMillis();

        if (drivePad.start) {
            if (controlPad != drivePad) {
                lastControlPad = controlPad;
                controlPad = drivePad;
            }
        } else if (drivePad.back) {
            controlPad = lastControlPad;
        }

        if (controlPad.a) gripperClosed = true;
        else if (controlPad.b) gripperClosed = false;
        hwcon.openCloseBlockGripper(gripperClosed);

        if (controlPad.left_bumper) armDown = false;
        else if (controlPad.right_bumper) armDown = true;
        hwcon.raiseLowerArm(armDown);

        hwcon.startRightWave(controlPad.dpad_right);

        hwcon.controlLift(controlPad.right_trigger - controlPad.left_trigger);

        double turnAmount = 0.0;
        /*
        if (turnStartTime > -1) {
            turnDuration = System.currentTimeMillis() - turnStartTime;
        }
        if (Math.abs(drivePad.right_stick_x) <= 0.01) {
            if (turnStartTime < 0) turnStartTime = System.currentTimeMillis();
            if (turnDuration >= turnWait) turnAmount = hwcon.getTurnPID(0);
            //turnAmount = turnToAngleGetTurn(0);
            //turnAmount = getTurnPID(0);
        }
        else {
            turnAmount = drivePad.right_stick_x;
            hwcon.setZeroedHeading();
            turnStartTime = -1;
        }*/
        turnAmount = drivePad.right_stick_x;

        telemetry.addData("Turn Amount", turnAmount);
        hwcon.MecanumDrive(drivePad.left_stick_x, drivePad.left_stick_y, turnAmount);

        //telemetry.addData("Right Ball", hwcon.getRightBallColor());
        //telemetry.addData("Left Ball", hwcon.getLeftBallColor());

        telemetry.addData("Last Loop Time", lastLoopTime);

        //telemetry.addData("Ultra Sonic", hwcon.getRangeUltraSonicValue());
        //telemetry.addData("ODS", hwcon.getRangeODSValue());

        // Update the sensors and the telemetry in the hardware controller
        hwcon.updateSensorsAndTelmetry();

        lastLoopTime = System.currentTimeMillis() - loopStart;
    }

    @Override
    public void stop() {
        hwcon.MecanumDrive(0,0,0);
    }
}

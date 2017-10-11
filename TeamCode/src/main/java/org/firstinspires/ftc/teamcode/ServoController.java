package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Michael on 10/5/2017.
 */

@TeleOp(name="ServoControl", group="test")
public class ServoController extends OpMode {

    private Servo controlServo1;
    private Servo controlServo2;

    private double servo1Position = 0.0, servo2Position = 0.0;

    // Button states
    private boolean lastAState=false, lastBState=false, lastXState=false, lastYState=false;

    @Override
    public void init() {
        controlServo1 = hardwareMap.get(Servo.class, "rightGrab");
        controlServo2 = hardwareMap.get(Servo.class, "leftGrab");
    }

    @Override
    public void loop() {
        boolean AState, BState, XState, YState;
        AState = gamepad1.a;
        BState = gamepad1.b;
        XState = gamepad1.x;
        YState = gamepad1.y;

        if (AState != lastAState) {
            lastAState = AState;
            if (AState) servo1Position += 0.1;
        } else if (BState != lastBState) {
            lastBState = BState;
            if (BState) servo1Position -= 0.1;
        }

        if (XState != lastXState) {
            lastXState = XState;
            if (XState) servo2Position += 0.1;
        } else if (YState != lastYState) {
            lastYState = YState;
            if (YState) servo2Position -= 0.1;
        }

        controlServo1.setPosition(servo1Position);
        controlServo2.setPosition(servo2Position);

        telemetry.addData("Servo1Position", servo1Position);
        telemetry.addData("Servo2Position", servo2Position);
    }
}

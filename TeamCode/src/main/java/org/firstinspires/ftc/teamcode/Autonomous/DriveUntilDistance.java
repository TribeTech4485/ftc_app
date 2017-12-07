package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SystemControl.HardwareController;

/**
 * Created by Michael on 11/30/2017.
 */

@Autonomous(name="BallAuto", group="MainAuto")
public class DriveUntilDistance extends LinearOpMode {

    HardwareController hwcon;

    @Override
    public void runOpMode() {
        hwcon = new HardwareController();
        hwcon.initHardware(hardwareMap, telemetry);

        hwcon.raiseLowerArm(true);

        while(opModeIsActive()) {
            telemetry.addData("UltraSonic Distance", hwcon.getRangeUltraSonicValue());
            hwcon.updateSensorsAndTelmetry();
        }
    }

}

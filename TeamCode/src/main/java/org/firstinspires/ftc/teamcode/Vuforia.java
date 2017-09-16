package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Michael on 9/14/2017.
 */


public class Vuforia extends LinearOpMode {
    VuforiaLocalizer vuforiaLocalizer;
    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables visionTargets;
    VuforiaTrackable target;
    VuforiaTrackableDefaultListener listener;

    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phoneLocation;

    public static final String VUFORIA_KEY = "ASbmTen/////AAAAGQTC3VAB+0PEueH1BQZKqdp2CV3QV5nUEsqSg1ygegLmqRygjsza4RmAL0sNg5Zv9GjtA/90KMalTxwFy8MbdM1AEqhqD6+hPXTYsZN1HMthQ3tcqH5f04ylRYjHkeTkz6c8l/ibCKRMl0VNLWfF2Fw+2SOmnDaPWT+jqbUTX2j+dMUONXnv8JgeNxBVmg+ytQfRQcM0uAJ9gNeTV051BQZKqdp2CV3QV5nUEsqSg1ygegLmqRygjNpfCPHAw3LkgTc46Dx634AhhjtOa4CdxbDeIYOu2yuJy0SLZl+u3GKZftwRq2AOktJ6itK7XLhVWsbyyrKJY/RggvbZ+0GbPmNnXw9L6+GS4ZFAkOiRNr"; // API Key

    public void runOpMode() throws InterruptedException {
        waitForStart();

        while(opModeIsActive()) {


            telemetry.update();
            idle();
        }
    }

    public void setupVuforia() {
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("RelicVuMark");

        target = visionTargets.get(0);
        target.setName("RelicTarget");
       // target.setLocation(createMatrix());
    }

}

package org.firstinspires.ftc.teamcode.PID;

/**
 * Created by Michael on 10/27/2017.
 */

public class PIDController {
    private SPID spid;

    double lastReturn = 0;

    // Delay for consistency
    private double delay = 10;  // 10 ms
    private double startTime = -1;
    private double duration = -1;

    public PIDController() {
        spid = new SPID();
        duration = delay;
        checkTime();
    }

    public void setSPID(SPID _spid) { spid = _spid; }
    public SPID getSPID() { return spid; }

    private boolean checkTime() {
        if (startTime < 0) startTime = System.currentTimeMillis();
        duration = System.currentTimeMillis() - startTime;
        if (duration >= delay) {
            duration = -1;
            startTime = -1;
            return true;
        }
        return false;
    }

    public double update(double error, double position) {
        if (!checkTime()) return lastReturn;

        double pTerm, dTerm, iTerm;

        pTerm = spid.pGain * error; // Calculate the proportional term

        spid.iState += error;
        if (spid.iState > spid.iMax) spid.iState = spid.iMax;
        if (spid.iState < spid.iMin) spid.iState = spid.iMin;
        iTerm = spid.iGain * spid.iState;

        dTerm = spid.dGain * (position - spid.dState);
        spid.dState = position;

        double returnVal = pTerm + iTerm - dTerm;
        lastReturn = returnVal;
        return returnVal;
    }
}

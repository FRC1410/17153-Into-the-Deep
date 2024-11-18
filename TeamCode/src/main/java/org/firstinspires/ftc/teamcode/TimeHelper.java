package org.firstinspires.ftc.teamcode;

import java.util.TimerTask;

public class TimeHelper extends TimerTask {

    public static int TIME = 0;

    public double getCurrentTime () {
        return TIME++;
    }

    @Override
    public void run() {
        this.getCurrentTime();
    }
}

package org.firstinspires.ftc.teamcode.Actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.Wrist;
import org.firstinspires.ftc.teamcode.Util.RobotStates;

public class LiftWrist {
    private final Wrist wrist;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean hasReset = false;

    public LiftWrist(Wrist wrist) {
        this.wrist = wrist;
    }

    public void liftWrist() {
        if (!hasReset) {
            this.timer.reset();
            hasReset = true;
        } else {
            this.wrist.setState(RobotStates.Wrist.SCORE);

            if(timer.seconds() > 1) {
                this.wrist.setState(RobotStates.Wrist.FLOOR);
                hasReset = false;
            }
        }
    }
}

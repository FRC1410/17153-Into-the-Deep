package org.firstinspires.ftc.teamcode.Actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.Arm;
import org.firstinspires.ftc.teamcode.Subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystem.Wrist;
import org.firstinspires.ftc.teamcode.Util.RobotStates;

public class Raise {
    private final LinearSlide linearSlide;
    private final Arm arm;
    private final Wrist wrist;

    private final ElapsedTime timer = new ElapsedTime();
    boolean hasReset = false;

    public Raise(LinearSlide linearSlide, Arm arm, Wrist wrist) {
        this.linearSlide = linearSlide;
        this.arm = arm;
        this.wrist = wrist;

        this.timer.reset();
    }

    public void raise() {
            this.arm.setState(RobotStates.Arm.UP);
            this.linearSlide.setState(RobotStates.LinearSlide.HIGH_SCORE);

            if (this.timer.seconds() >= 0.5) {
                wrist.setState(RobotStates.Wrist.SCORE);
        }
    }
}

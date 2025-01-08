package org.firstinspires.ftc.teamcode.Actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.Shoulder;
import org.firstinspires.ftc.teamcode.Subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystem.Wrist;
import org.firstinspires.ftc.teamcode.Util.RobotStates;

public class Safety {
    private final LinearSlide linearSlide;
    private final Shoulder shoulder;
    private final Wrist wrist;

    private final ElapsedTime timer = new ElapsedTime();

    public Safety(LinearSlide linearSlide, Shoulder shoulder, Wrist wrist) {
        this.linearSlide = linearSlide;
        this.shoulder = shoulder;
        this.wrist = wrist;

//        LinearSlide.hasReachedState = false;
    }

    public void safe() {
        this.wrist.setState(RobotStates.Wrist.SAFE);
    }
}

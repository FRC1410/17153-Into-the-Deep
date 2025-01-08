package org.firstinspires.ftc.teamcode.Actions;

import org.firstinspires.ftc.teamcode.Subsystem.Shoulder;
import org.firstinspires.ftc.teamcode.Subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.Util.RobotStates;

public class InitClimb {
    private final LinearSlide linearSlide;
    private final Shoulder shoulder;

    public InitClimb(LinearSlide linearSlide, Shoulder shoulder) {
        this.linearSlide = linearSlide;
        this.shoulder = shoulder;
    }

    public void climb() {
        //this.wrist.setState(RobotStates.Wrist.SCORE);
        this.shoulder.setState(RobotStates.Arm.CLIMB_INIT);
        if(Shoulder.hasReachedState) {
            this.linearSlide.setState(RobotStates.LinearSlide.CLIMB);
        }

    }
}
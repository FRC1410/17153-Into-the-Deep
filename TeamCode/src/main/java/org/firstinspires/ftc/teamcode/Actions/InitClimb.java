package org.firstinspires.ftc.teamcode.Actions;

import org.firstinspires.ftc.teamcode.Subsystem.Arm;
import org.firstinspires.ftc.teamcode.Subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystem.Wrist;
import org.firstinspires.ftc.teamcode.Util.RobotStates;

public class InitClimb {
    private final LinearSlide linearSlide;
    private final Arm arm;

    public InitClimb(LinearSlide linearSlide, Arm arm) {
        this.linearSlide = linearSlide;
        this.arm = arm;
    }

    public void climb() {
        //this.wrist.setState(RobotStates.Wrist.SCORE);
        this.arm.setState(RobotStates.Arm.CLIMB_INIT);
        if(Arm.hasReachedState) {
            this.linearSlide.setState(RobotStates.LinearSlide.CLIMB);
        }

    }
}
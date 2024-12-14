package org.firstinspires.ftc.teamcode.Actions;

import org.firstinspires.ftc.teamcode.Subsystem.Arm;
import org.firstinspires.ftc.teamcode.Subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystem.Wrist;
import org.firstinspires.ftc.teamcode.Util.RobotStates;

public class DoClimb {
    private final LinearSlide linearSlide;
    private final Arm arm;
    private final Wrist wrist;

    public DoClimb(LinearSlide linearSlide, Arm arm, Wrist wrist) {
        this.linearSlide = linearSlide;
        this.arm = arm;
        this.wrist = wrist;

    }

    public void climb() {
        //this.wrist.setState(RobotStates.Wrist.SCORE);
            this.arm.setState(RobotStates.Arm.DOWN);
            if(Arm.hasReachedState) {
                this.linearSlide.setState(RobotStates.LinearSlide.START_POS);
            }
        }

    }

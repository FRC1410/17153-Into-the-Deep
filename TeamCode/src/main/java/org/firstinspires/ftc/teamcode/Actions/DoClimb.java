package org.firstinspires.ftc.teamcode.Actions;

import org.firstinspires.ftc.teamcode.Subsystem.Shoulder;
import org.firstinspires.ftc.teamcode.Subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.Util.RobotStates;

public class DoClimb {
    private final LinearSlide linearSlide;
    private final Shoulder shoulder;

    public DoClimb(LinearSlide linearSlide, Shoulder shoulder) {
        this.linearSlide = linearSlide;
        this.shoulder = shoulder;
    }

    public void climb() {
            this.shoulder.setState(RobotStates.Arm.DOWN);
            this.linearSlide.setState(RobotStates.LinearSlide.START_POS);
        }
    }

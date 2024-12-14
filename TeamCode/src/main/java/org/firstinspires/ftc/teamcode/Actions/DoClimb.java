package org.firstinspires.ftc.teamcode.Actions;

import org.firstinspires.ftc.teamcode.Subsystem.Arm;
import org.firstinspires.ftc.teamcode.Subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystem.Wrist;
import org.firstinspires.ftc.teamcode.Util.RobotStates;

public class DoClimb {
    private final LinearSlide linearSlide;
    private final Arm arm;

    public DoClimb(LinearSlide linearSlide, Arm arm) {
        this.linearSlide = linearSlide;
        this.arm = arm;
    }

    public void climb() {
            this.arm.setState(RobotStates.Arm.DOWN);
            this.linearSlide.setState(RobotStates.LinearSlide.START_POS);
        }
    }

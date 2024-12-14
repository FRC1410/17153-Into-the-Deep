package org.firstinspires.ftc.teamcode.Actions;

import com.acmerobotics.roadrunner.Line;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.Arm;
import org.firstinspires.ftc.teamcode.Subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystem.Wrist;
import org.firstinspires.ftc.teamcode.Util.RobotStates;

public class Climb {
    private final LinearSlide linearSlide;
    private final Arm arm;
    private final Wrist wrist;

    public Climb(LinearSlide linearSlide, Arm arm, Wrist wrist) {
        this.linearSlide = linearSlide;
        this.arm = arm;
        this.wrist = wrist;

    }

    public void climb() {
        this.arm.setState(RobotStates.Arm.CLIMB_INIT);
        if(Arm.hasReachedState) {
            this.linearSlide.setState(RobotStates.LinearSlide.CLIMB);
            if(LinearSlide.hasReachedState) {
                this.arm.setState(RobotStates.Arm.CLIMB_FINAL);
                if(Arm.hasReachedState) {
                    this.linearSlide.setState(RobotStates.LinearSlide.START_POS);
                }
            }
        }

    }
}
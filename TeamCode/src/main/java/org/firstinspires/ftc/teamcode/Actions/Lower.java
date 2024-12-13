package org.firstinspires.ftc.teamcode.Actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.Arm;
import org.firstinspires.ftc.teamcode.Subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystem.Wrist;
import org.firstinspires.ftc.teamcode.Util.RobotStates;

public class Lower {
    private final LinearSlide linearSlide;
    private final Arm arm;
    private final Wrist wrist;

    public Lower(LinearSlide linearSlide, Arm arm, Wrist wrist) {
        this.linearSlide = linearSlide;
        this.arm = arm;
        this.wrist = wrist;

//        LinearSlide.hasReachedState = false;
    }

    public void lower() {
            this.linearSlide.setState(RobotStates.LinearSlide.START_POS);
            this.wrist.setState(RobotStates.Wrist.FLOOR);

            if(LinearSlide.hasReachedState) {
                this.arm.setState(RobotStates.Arm.DOWN);
            }
        }
}

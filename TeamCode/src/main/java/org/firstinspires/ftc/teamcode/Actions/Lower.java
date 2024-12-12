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
    private final ElapsedTime timer = new ElapsedTime();
    boolean hasReset = false;

    public Lower(LinearSlide linearSlide, Arm arm, Wrist wrist) {
        this.linearSlide = linearSlide;
        this.arm = arm;
        this.wrist = wrist;

        this.timer.reset();
    }

    public void lower() {
            this.linearSlide.setState(RobotStates.LinearSlide.START_POS);

            if (this.timer.seconds() > 1.1) {
                this.wrist.setState(RobotStates.Wrist.SCORE);
                this.arm.setState(RobotStates.Arm.DOWN);
            }

            if(this.timer.seconds() > 2) {
                this.arm.setState(RobotStates.Arm.DOWN);
            }
        }
}

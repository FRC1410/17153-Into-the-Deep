package org.firstinspires.ftc.teamcode.Util;

public class AutoStates {
    public enum Arm {
        DOWN,
        UP,
        CLIMB_INIT,

    }

    public enum Claw {
        OPEN,
        CLOSED
    }

    public enum LinearSlide {
        START_POS,
        MANUEL,
        HIGH_SCORE,
        LOW_SCORE,
        CLIMB
    }

    public enum Wrist {
        FLOOR,
        SCORE,
        SAFE
    }

    public enum Drivetrain {
        FULL_SPEED,
        HALF_SPEED
    }
}


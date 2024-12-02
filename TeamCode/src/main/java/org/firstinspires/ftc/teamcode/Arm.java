package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    private DcMotorEx armMotor;
    private final double TICKS_PER_DEGREE = 16;
    private final double HOLD_POWER = 0.1;
    private int targetPosition;

    public void init(HardwareMap hardwareMap) {
        this.armMotor = hardwareMap.get(DcMotorEx.class, "4");
        this.armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public boolean moveToPosition(double degrees) {
        targetPosition = (int) (degrees * TICKS_PER_DEGREE);
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.5);

        return Math.abs(armMotor.getCurrentPosition() - targetPosition) < 10;
    }

    public void holdPosition() {
        armMotor.setPower(HOLD_POWER);
    }
}

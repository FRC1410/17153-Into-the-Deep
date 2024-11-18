package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Arm {
    private DcMotorEx armMotor;

    public void init(HardwareMap hardwareMap) {
        this.armMotor = hardwareMap.get(DcMotorEx.class, "1");
    }

    public void runArmMotor(double power) {
        double clampedPower = Range.clip(power, -1, 1);

        this.armMotor.setPower(clampedPower);
    }
}

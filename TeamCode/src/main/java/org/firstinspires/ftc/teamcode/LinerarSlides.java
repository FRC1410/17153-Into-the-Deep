package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class LinerarSlides {
    private DcMotorEx leftLinearSlide;
    private DcMotorEx rightLinearSlide;

    public void init(HardwareMap hardwareMap) {
        this.leftLinearSlide = hardwareMap.get(DcMotorEx.class, "7");
        this.rightLinearSlide = hardwareMap.get(DcMotorEx.class, "6");

        this.leftLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void slidePower(double power) {
        double clampedPower = Range.clip(power, -1, 1);

        this.leftLinearSlide.setPower(clampedPower);
        this.rightLinearSlide.setPower(clampedPower);
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class Robot extends OpMode {
    private final Arm arm = new Arm();


    @Override
    public void init() {
        this.arm.init(hardwareMap);
    }

    @Override
    public void loop() {
        this.arm.runArmMotor(gamepad1.left_stick_x);
    }
}
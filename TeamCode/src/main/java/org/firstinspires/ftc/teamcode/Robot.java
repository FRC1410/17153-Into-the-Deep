package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class Robot extends OpMode {

    Drivetrain drivetrain = new Drivetrain();


    @Override
    public void init() {
        drivetrain.init(hardwareMap);
    }

    @Override
    public void loop() {
        drivetrain.loop(gamepad1.left_stick_y,gamepad1.right_stick_x, gamepad1.left_stick_x);
    }
}
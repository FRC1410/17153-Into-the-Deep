package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Robot extends OpMode {

    Drivetrain drivetrain = new Drivetrain();


    @Override
    public void init() {
        drivetrain.init();
    }

    @Override
    public void loop() {
        drivetrain.loop();
    }
}
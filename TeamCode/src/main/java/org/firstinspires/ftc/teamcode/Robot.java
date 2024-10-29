package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Robot extends OpMode {

    servoClaw servoClaw = new servoClaw();
    servoWrist servoWrist = new servoWrist();

    @Override
    public void init() {
        servoClaw.init(hardwareMap);
        servoWrist.init(hardwareMap);
    }

    @Override
    public void loop() {
        servoClaw.runServoClaw(gamepad1.a, gamepad1.b);
        servoWrist.runServoWrist(gamepad1.x, gamepad1.y);
    }
}
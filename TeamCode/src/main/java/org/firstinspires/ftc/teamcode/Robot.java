package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.constants.clawOpenValue;
import static org.firstinspires.ftc.teamcode.constants.wristOpenValue;

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
        if (gamepad1.a) {
            servoClaw.runServoClaw(clawOpenValue);
        }
        if (gamepad1.a) {
            servoWrist.runServoWrist(wristOpenValue);
        }
        //These are gamepad1 for testing, will change to gamepad2
    }
}
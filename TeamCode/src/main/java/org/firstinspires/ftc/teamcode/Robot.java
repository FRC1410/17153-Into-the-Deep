package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.constants.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Robot extends OpMode {

    Claw Claw = new Claw();
    ClawWrist Wrist = new ClawWrist();

    @Override
    public void init() {
        Claw.init(hardwareMap);
        Wrist.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            Claw.setServoClawPos(openClawValue);
        } else {
            Claw.setServoClawPos(closedClawValue);
        }

        if (gamepad1.b) {
            Wrist.setServoWristPos(openWristValue);
        } else {
            Wrist.setServoWristPos(closedWristValue);
        }
        //These are gamepad1 for testing, will change to gamepad2
    }
}
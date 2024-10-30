package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    public Servo servoClaw;

    public void setServoClawPos(double clawInput) {

        servoClaw.setPosition(clawInput); //take input

    }

    public void init(HardwareMap hardwareMap) {

        servoClaw = hardwareMap.get(Servo.class,"servoClawPosSet");

    }

}
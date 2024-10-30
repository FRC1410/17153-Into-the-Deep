package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.constants.wristCloseValue;
import static org.firstinspires.ftc.teamcode.constants.wristOpenValue;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servoWrist {
    public Servo servoWrist;

    public void runServoWrist(double wristInput) {

        servoWrist.setPosition(wristInput); // open

    }

    public void init(HardwareMap hardwareMap) {

        servoWrist = hardwareMap.get(Servo.class,"servoWrist");

    }

}
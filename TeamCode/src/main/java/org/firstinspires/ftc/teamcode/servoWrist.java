package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servoWrist {
    public Servo servoWrist;

    public void runServoWrist(boolean c, boolean d) {

        if (c){
            servoWrist.setPosition(1);
        }
        else if(d){
            servoWrist.setPosition(0.4);
        }

    }

    public void init(HardwareMap hardwareMap) {

        servoWrist = hardwareMap.get(Servo.class,"servoWrist");

    }

}
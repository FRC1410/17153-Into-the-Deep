package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servoWrist {
    public Servo servoWrist;

    public void runServoWrist(boolean x, boolean y) {

        if (x){
            servoWrist.setPosition(1); // open
        }
        else if(y){
            servoWrist.setPosition(0.4); // closed
        }

    }

    public void init(HardwareMap hardwareMap) {

        servoWrist = hardwareMap.get(Servo.class,"servoWrist");

    }

}
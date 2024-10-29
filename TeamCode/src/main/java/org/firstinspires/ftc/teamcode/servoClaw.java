package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servoClaw {
    public Servo servoClaw;

    public void runServoClaw(boolean a, boolean b) {

        if (a){
            servoClaw.setPosition(1);
        }
        else if(b){
            servoClaw.setPosition(0.4);
        }

    }

    public void init(HardwareMap hardwareMap) {

        servoClaw = hardwareMap.get(Servo.class,"servoClaw");

    }

}
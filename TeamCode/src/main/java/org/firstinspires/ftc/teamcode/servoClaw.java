package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.constants.clawCloseValue;
import static org.firstinspires.ftc.teamcode.constants.clawOpenValue;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servoClaw {
    public Servo servoClaw;

    public void runServoClaw(boolean openClaw, boolean closeClaw) {

        if (openClaw){
            servoClaw.setPosition(clawOpenValue); //open
        }
        else if(closeClaw){
            servoClaw.setPosition(clawCloseValue); //closed
        }

    }

    public void init(HardwareMap hardwareMap) {

        servoClaw = hardwareMap.get(Servo.class,"servoClaw");

    }

}
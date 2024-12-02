package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Robot extends OpMode {
    private final Arm arm = new Arm();
    private States armState = States.IDLE;
    private boolean wasXPressed = false;

    @Override
    public void init() {
        this.arm.init(hardwareMap);
    }

    @Override
    public void loop() {
        boolean isXPressed = gamepad1.x;

    
        if (isXPressed && !wasXPressed) {
            
            if (armState == States.IDLE) {
                armState = States.MOVE_TO_POSITION;
            } else if (armState == States.HOLD_POSITION) {
                armState = States.IDLE;
            }
        }
        wasXPressed = isXPressed;

        
        switch (armState) {
            case IDLE:
                
                break;

            case MOVE_TO_POSITION:
                
                if (arm.moveToPosition(45)) {
                    armState = States.HOLD_POSITION;
                }
                break;

            case HOLD_POSITION:
                
                arm.holdPosition();
                break;
        }
    }
}

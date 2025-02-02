package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.RobotStates;

public class AutoWrist {
    private ServoImplEx wristServo;

    private RobotStates.Wrist currentWristState = RobotStates.Wrist.SCORE;
    private double wristPos;
    public static boolean hasReachedState = false;

    public void init(HardwareMap hardwareMap) {
        this.wristServo = hardwareMap.get(ServoImplEx.class, "servoWristPosSet");
        this.wristServo.setDirection(Servo.Direction.FORWARD);
        reset(); // Reset wrist position on initialization
    }

    public void reset() {
        // Set the wrist to the SAFE position when resetting
        setState(RobotStates.Wrist.FLOOR);
        goToState();
        hasReachedState = true; // Assume it's already in position after reset
    }
    public void setState(RobotStates.Wrist desiredWristState) {
        this.currentWristState = desiredWristState;
        hasReachedState = false; // Reset state tracking
    }

    public RobotStates.Wrist getCurrentState() {
        return currentWristState;
    }

    private void getDesiredWristPos(RobotStates.Wrist desiredWristState) {
        switch (desiredWristState) {
            case FLOOR:
                this.wristPos = 0.95;
                break;
            case SCORE:
                this.wristPos = 0;
                break;
            case SAFE:
                this.wristPos = 0.5;
                break;
        }
    }

    public void goToState() {
        RobotStates.Wrist desiredWristState = this.getCurrentState();
        this.getDesiredWristPos(desiredWristState);

        // Continuously adjust position until it reaches the desired value
        if (Math.abs(wristServo.getPosition() - wristPos) > 0.01) {
            wristServo.setPosition(wristPos);
        } else {
            hasReachedState = true; // Mark as reached once in position
        }
    }

    public int getWristPos() {
        return (int) this.wristServo.getPosition();
    }

    public boolean getCompletion(){
        return hasReachedState;
    }


    public void wristTelemetry(Telemetry telemetry) {
        telemetry.addData("Wrist State: ", this.currentWristState);
        telemetry.addData("Wrist Position: ", this.wristServo.getPosition());
        telemetry.addData("Desired Position: ", this.wristPos);
    }
}

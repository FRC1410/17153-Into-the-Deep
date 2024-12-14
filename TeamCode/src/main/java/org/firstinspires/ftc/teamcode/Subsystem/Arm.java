package org.firstinspires.ftc.teamcode.Subsystem;

import static org.firstinspires.ftc.teamcode.Util.IDs.ARM_MOTOR_ID;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.PIDController;
import org.firstinspires.ftc.teamcode.Util.RobotStates;

import static org.firstinspires.ftc.teamcode.Util.Tuning.*;

public class Arm {
    public static boolean hasReachedState;
    private DcMotorEx armMotor;
    private final PIDController armPIDController = new PIDController(ARM_P, ARM_I, ARM_D);
    private RobotStates.Arm currentArmState = RobotStates.Arm.DOWN;

    private int desiredAngle;

    public void init(HardwareMap hardwareMap) {
        this.armMotor = hardwareMap.get(DcMotorEx.class, ARM_MOTOR_ID);

        this.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setState(RobotStates.Arm desiredState) {
        currentArmState = desiredState;
    }

    public RobotStates.Arm getArmState() {
        return currentArmState;
    }

    public void setArmAngle(RobotStates.Arm desiredState) {
        switch (desiredState) {
            case DOWN:
                desiredAngle = 0;
                break;

            case UP:
                desiredAngle = 1700;
                break;

            case CLIMB_INIT:
                desiredAngle = 810;
                break;
        }
    }

    public void goToState() {
        RobotStates.Arm desiredState = this.getArmState();
        this.setArmAngle(desiredState);
        int currentArmPos = this.armMotor.getCurrentPosition();

        double armOutput = armPIDController.calculate(desiredAngle, currentArmPos);

        this.armMotor.setPower(armOutput);

        if(Math.abs(desiredAngle - currentArmPos) <= ARM_THRESHOLD) {
            this.armMotor.setPower(0);
            hasReachedState = true;
        }
    }

    public void armTelemetry(Telemetry telemetry) {
        telemetry.addData("ArmEncoder", this.armMotor.getCurrentPosition());
    }
}

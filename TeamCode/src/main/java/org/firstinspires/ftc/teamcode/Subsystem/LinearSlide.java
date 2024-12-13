package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.RobotStates;
import org.firstinspires.ftc.teamcode.Util.PIDController;

import static org.firstinspires.ftc.teamcode.Util.IDs.*;
import static org.firstinspires.ftc.teamcode.Util.Tuning.*;
import static org.firstinspires.ftc.teamcode.Util.Constants.*;

public class LinearSlide {
    private DcMotorEx leftSlideMotor;
    private DcMotorEx rightSlideMotor;
    private VoltageSensor controlHubVoltageSensor;

    private PIDController leftPIDController = new PIDController(LINEAR_SLIDE_P, LINEAR_SLIDE_I, LINEAR_SLIDE_D);
    private PIDController rightPIDController = new PIDController(LINEAR_SLIDE_P, LINEAR_SLIDE_I, LINEAR_SLIDE_D);

    private double leftOutput;
    private double rightOutput;

    private RobotStates.LinearSlide currentSlideState = RobotStates.LinearSlide.MANUEL;
    private int desiredSlideHeight;
    private int customHeight;

    public static boolean hasReachedState;

    public void init(HardwareMap hardwareMap) {
        this.leftSlideMotor = hardwareMap.get(DcMotorEx.class, LEFT_SLIDE_MOTOR_ID);
        this.rightSlideMotor = hardwareMap.get(DcMotorEx.class, RIGHT_SLIDE_MOTOR_ID);

        this.leftSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.rightSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        this.leftSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.rightSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        this.leftSlideMotor.setDirection(DcMotorEx.Direction.REVERSE);
        this.rightSlideMotor.setDirection(DcMotorEx.Direction.FORWARD);

        this.leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.controlHubVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        this.leftPIDController = new PIDController(LINEAR_SLIDE_P, LINEAR_SLIDE_I, LINEAR_SLIDE_D);
        this.rightPIDController = new PIDController(LINEAR_SLIDE_P, LINEAR_SLIDE_I, LINEAR_SLIDE_D);
    }

    public RobotStates.LinearSlide getCurrentState() {
        return currentSlideState;
    }
    public void setState(RobotStates.LinearSlide desiredState) {
        currentSlideState = desiredState;
    }

    public void setDesiredSlideHeight(RobotStates.LinearSlide linearSlideState) {
        switch (linearSlideState) {
            case START_POS:
                desiredSlideHeight = 0;
                break;

            case MANUEL:
                desiredSlideHeight = customHeight;
                break;

            case HIGH_SCORE:
                desiredSlideHeight = 2_900;
                break;
        }
    }

    public void goToState(int leftVal, int rightVal) {
        hasReachedState = false;
        RobotStates.LinearSlide desiredState = this.getCurrentState();

        this.setDesiredSlideHeight(desiredState);

        int currentLeftPos = this.leftSlideMotor.getCurrentPosition();
        int currentRightPos = this.rightSlideMotor.getCurrentPosition();

        if (desiredState == RobotStates.LinearSlide.MANUEL) {
            if (customHeight > MAX_LINEAR_SLIDE_EXTENSION) {
                customHeight = MAX_LINEAR_SLIDE_EXTENSION;
            }

            if(customHeight < 0) {
                customHeight = 0;
            }

            customHeight += (rightVal - leftVal) * 100;

            this.leftOutput = this.leftPIDController.calculate(customHeight, currentLeftPos);
            this.rightOutput = this.rightPIDController.calculate(customHeight, currentRightPos);

            if(Math.abs(customHeight - currentLeftPos) <= LINEAR_SLIDE_THRESHOLD) {
                this.leftSlideMotor.setPower(0);
            }

            if(Math.abs(customHeight - currentRightPos) <= LINEAR_SLIDE_THRESHOLD) {
                this.rightSlideMotor.setPower(0);
            }

        } else {
            this.leftOutput = this.leftPIDController.calculate(desiredSlideHeight, currentLeftPos);
            this.rightOutput = this.rightPIDController.calculate(desiredSlideHeight, currentRightPos);
        }

        this.leftSlideMotor.setPower(leftOutput);
        this.rightSlideMotor.setPower(rightOutput);

        if(Math.abs(desiredSlideHeight - currentLeftPos) <= LINEAR_SLIDE_THRESHOLD) {
            this.leftSlideMotor.setPower(0);
            hasReachedState = true;
        }
        if(Math.abs(desiredSlideHeight - currentRightPos) <= LINEAR_SLIDE_THRESHOLD) {
            this.rightSlideMotor.setPower(0);
        }
    }

    public void setSlidePower(float leftTrigger, float rightTrigger) {
        double voltageCorrection = 12 / controlHubVoltageSensor.getVoltage();

        this.leftSlideMotor.setPower((rightTrigger - leftTrigger) * voltageCorrection);
        this.rightSlideMotor.setPower((rightTrigger - leftTrigger) * voltageCorrection);
    }

    public void slideData(Telemetry telemetry) {
        telemetry.addData("Left Slide Encoder: ", this.leftSlideMotor.getCurrentPosition());
        telemetry.addData("Right Slide Encoder: ", this.rightSlideMotor.getCurrentPosition());
    }
}

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Constants.CONTROL_HUB;
import static org.firstinspires.ftc.teamcode.Constants.FRONT_LEFT_MOTOR_ID;
import static org.firstinspires.ftc.teamcode.Constants.BACK_LEFT_MOTOR_ID;
import static org.firstinspires.ftc.teamcode.Constants.FRONT_RIGHT_MOTOR_ID;
import static org.firstinspires.ftc.teamcode.Constants.BACK_RIGHT_MOTOR_ID;


public class Drivetrain {

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    VoltageSensor controlHubVoltageSensor;

    double[] wheelSpeeds = new double[4];

    public void init(HardwareMap hardwareMap) {

        motorFL = hardwareMap.dcMotor.get(FRONT_LEFT_MOTOR_ID);
        motorBL = hardwareMap.dcMotor.get(BACK_LEFT_MOTOR_ID);
        motorFR = hardwareMap.dcMotor.get(FRONT_RIGHT_MOTOR_ID);
        motorBR = hardwareMap.dcMotor.get(BACK_RIGHT_MOTOR_ID);

        motorFL.setDirection(FORWARD);
        motorFR.setDirection(FORWARD);
        motorBL.setDirection(FORWARD);
        motorBR.setDirection(FORWARD);

        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, CONTROL_HUB);
    }

    public void mechanumDrive(
            double strafeSpeed,
            double forwardSpeed,
            double turnSpeed,
            double gyroAngle)
    {
        Vector2d input = new Vector2d(strafeSpeed, forwardSpeed);

        wheelSpeeds[0] = forwardSpeed + strafeSpeed + turnSpeed;
        wheelSpeeds[1] = forwardSpeed - strafeSpeed - turnSpeed;
        wheelSpeeds[2] = forwardSpeed - strafeSpeed + turnSpeed;
        wheelSpeeds[3] = forwardSpeed + strafeSpeed - turnSpeed;

        double voltageCorrection = 12 / controlHubVoltageSensor.getVoltage();

        for(int i = 0; i < wheelSpeeds.length; i++) {
            wheelSpeeds[i] = Math.abs(wheelSpeeds[i]) < 0.01 ?
                    wheelSpeeds[i] * voltageCorrection :
                    (wheelSpeeds[i] + Math.signum(wheelSpeeds[i]) * 0.085) * voltageCorrection;
        }
    }
}




package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.Constants.FRONT_LEFT_MOTOR_ID;
import static org.firstinspires.ftc.teamcode.Constants.BACK_LEFT_MOTOR_ID;
import static org.firstinspires.ftc.teamcode.Constants.FRONT_RIGHT_MOTOR_ID;
import static org.firstinspires.ftc.teamcode.Constants.BACK_RIGHT_MOTOR_ID;


public class Drivetrain {

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

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
    }

    public void mechanumDrive(
            double strafeSpeed,
            double forwardSpeed,
            double turnSpeed,
            double gyroAngle)
    {
        Vector2d input = new Vector2d(strafeSpeed, forwardSpeed).rotate(-gyroAngle);

        wheelSpeeds[motorFL.setTargetPosition(forwardSpeed + strafeSpeed + turnSpeed);]

    }

}




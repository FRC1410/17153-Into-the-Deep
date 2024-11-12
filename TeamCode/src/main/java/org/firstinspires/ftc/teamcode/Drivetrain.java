package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.IDs.*;
import static org.firstinspires.ftc.teamcode.Constants.*;


public class Drivetrain {

    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;

    private VoltageSensor controlHubVoltageSensor;
    private IMU imu;

    double[] wheelSpeeds = new double[4];
    double maxPower = 1;

    public void init(HardwareMap hardwareMap) {

        this.motorFL = hardwareMap.dcMotor.get(FRONT_LEFT_MOTOR_ID);
        this.motorBL = hardwareMap.dcMotor.get(BACK_LEFT_MOTOR_ID);
        this.motorFR = hardwareMap.dcMotor.get(FRONT_RIGHT_MOTOR_ID);
        this.motorBR = hardwareMap.dcMotor.get(BACK_RIGHT_MOTOR_ID);

        this.motorFL.setDirection(FORWARD);
        this.motorFR.setDirection(FORWARD);
        this.motorBL.setDirection(FORWARD);
        this.motorBR.setDirection(REVERSE);

        this.motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.controlHubVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.imu = hardwareMap.get(IMU.class, CONTROL_HUB_IMU);
        this.imu.initialize(new IMU.Parameters(HUB_ORIENTATION));
    }

    public void mechanumDrive(
            double strafeSpeed,
            double forwardSpeed,
            double turnSpeed)
    {

        Vector2d input = new Vector2d(strafeSpeed, forwardSpeed);

        strafeSpeed = Range.clip(input.x, -1, 1);
        forwardSpeed = Range.clip(input.y, -1, 1);
        turnSpeed = Range.clip(turnSpeed, -1, 1);

        this.wheelSpeeds[0] = forwardSpeed + strafeSpeed + turnSpeed;
        this.wheelSpeeds[1] = forwardSpeed - strafeSpeed - turnSpeed;
        this.wheelSpeeds[2] = forwardSpeed - strafeSpeed + turnSpeed;
        this.wheelSpeeds[3] = forwardSpeed + strafeSpeed - turnSpeed;

        double voltageCorrection = 12 / controlHubVoltageSensor.getVoltage();

        for(int i = 0; i < this.wheelSpeeds.length; i++) {
            this.wheelSpeeds[i] = Math.abs(this.wheelSpeeds[i]) < 0.01 ?
                    this.wheelSpeeds[i] * voltageCorrection :
                    (this.wheelSpeeds[i] + Math.signum(this.wheelSpeeds[i]) * 0.085) * voltageCorrection;
        }

        for(double wheelSpeeds : wheelSpeeds) maxPower = Math.max(maxPower, Math.abs(wheelSpeeds));

        if(maxPower > 1) {
            this.wheelSpeeds[0] /= maxPower;
            this.wheelSpeeds[1] /= maxPower;
            this.wheelSpeeds[2] /= maxPower;
            this.wheelSpeeds[3] /= maxPower;
        }

        this.motorFL.setPower(wheelSpeeds[0]);
        this.motorFR.setPower(wheelSpeeds[1]);
        this.motorBL.setPower(wheelSpeeds[2]);
        this.motorBR.setPower(wheelSpeeds[3]);
    }
}
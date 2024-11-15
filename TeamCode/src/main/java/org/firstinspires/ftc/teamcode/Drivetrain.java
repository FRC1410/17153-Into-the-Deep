package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.IDs.*;
import static org.firstinspires.ftc.teamcode.Constants.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain {

    private DcMotorEx motorFL;
    private DcMotorEx motorFR;
    private DcMotorEx motorBL;
    private DcMotorEx motorBR;

    private VoltageSensor controlHubVoltageSensor;
    private IMU imu;
    private double[] wheelSpeeds = new double[4];
    private double maxPower = 1;

    public void init(HardwareMap hardwareMap) {

        this.motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.motorFL = hardwareMap.get(DcMotorEx.class, FRONT_LEFT_MOTOR_ID);
        this.motorBL = hardwareMap.get(DcMotorEx.class, BACK_LEFT_MOTOR_ID);
        this.motorFR = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT_MOTOR_ID);
        this.motorBR = hardwareMap.get(DcMotorEx.class, BACK_RIGHT_MOTOR_ID);

        this.motorFL.setDirection(FORWARD);
        this.motorFR.setDirection(FORWARD);
        this.motorBL.setDirection(FORWARD);
        this.motorBR.setDirection(REVERSE);

        this.motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

    // These parameters are in Inches per second
    public void autoDriveStraight(double forwardDistance, double velocity) {
        double encoderDistance = this.distanceToEncoderCount(forwardDistance);
        double encoderVelocity = this.velocityToEncoderCount(velocity);

        int currentFLPosition = this.motorFL.getCurrentPosition();
        int currentFRPosition = this.motorFR.getCurrentPosition();
        int currentBLPosition = this.motorBL.getCurrentPosition();
        int currentBRPosition = this.motorBR.getCurrentPosition();

        this.motorFL.setTargetPosition(currentFLPosition + (int) encoderDistance);
        this.motorFR.setTargetPosition(currentFRPosition + (int) encoderDistance);
        this.motorBL.setTargetPosition(currentBLPosition + (int) encoderDistance);
        this.motorBR.setTargetPosition(currentBRPosition + (int) encoderDistance);

        this.motorFL.setVelocity(encoderVelocity);
        this.motorBL.setVelocity(encoderVelocity);
        this.motorFR.setVelocity(encoderVelocity);
        this.motorBR.setVelocity(encoderVelocity);
    }

    public double distanceToEncoderCount(double desiredDistanceInInches) {
        return desiredDistanceInInches * ENCODER_COUNT_PER_INCH;
    }

    public double velocityToEncoderCount(double desiredVelocity) {
        double desiredRPM = (60 * desiredVelocity) / (GEAR_RATIO * WHEEL_CIRCUFERENCE);
        double ticksPerSecond = (desiredRPM / 60) * ENCODER_COUNT_PER_WHEEL_REVOLUTION;

        return ticksPerSecond;
    }

    public void drivetrainData(Telemetry telemetry) {
        telemetry.addData("Front Left: ", this.motorFL.getCurrentPosition());
        telemetry.addData("Front Right: ", this.motorFR.getCurrentPosition());
        telemetry.addData("Back Left: ", this.motorBL.getCurrentPosition());
        telemetry.addData("Back Right: ", this.motorBR.getCurrentPosition());
        telemetry.update();
    }
}
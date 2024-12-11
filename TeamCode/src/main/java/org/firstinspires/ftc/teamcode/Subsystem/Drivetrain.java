package org.firstinspires.ftc.teamcode.Subsystem;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.Util.IDs.*;
import static org.firstinspires.ftc.teamcode.Util.Constants.*;

import org.firstinspires.ftc.teamcode.Util.RobotStates;

public class Drivetrain {

    private DcMotorEx motorFL;
    private DcMotorEx motorFR;
    private DcMotorEx motorBL;
    private DcMotorEx motorBR;
    private VoltageSensor controlHubVoltageSensor;
    private IMU imu;

    private double[] wheelSpeeds = new double[4];
    private double maxPower = 1;

    private RobotStates.Drivetrain currentDrivetrainState = RobotStates.Drivetrain.FULL_SPEED;

    public void init(HardwareMap hardwareMap) {

        this.motorFL = hardwareMap.get(DcMotorEx.class, FRONT_LEFT_MOTOR_ID);
        this.motorBL = hardwareMap.get(DcMotorEx.class, BACK_LEFT_MOTOR_ID);
        this.motorFR = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT_MOTOR_ID);
        this.motorBR = hardwareMap.get(DcMotorEx.class, BACK_RIGHT_MOTOR_ID);

        this.motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        this.imu.resetYaw();
    }

    public RobotStates.Drivetrain getDrivetrainState() {
        return currentDrivetrainState;
    }

    public void setState(RobotStates.Drivetrain desiredState) {
        currentDrivetrainState = desiredState;
    }

    public void mechanumDrive(
            double strafeSpeed,
            double forwardSpeed,
            double turnSpeed,
            boolean switchMode)
    {

        Vector2d input = new Vector2d(strafeSpeed, forwardSpeed);

        strafeSpeed = Range.clip(input.x, -1, 1);
        forwardSpeed = Range.clip(input.y, -1, 1);
        turnSpeed = Range.clip(turnSpeed, -1, 1);

        this.wheelSpeeds[0] = forwardSpeed - strafeSpeed - turnSpeed;
        this.wheelSpeeds[1] = -forwardSpeed - strafeSpeed - turnSpeed;
        this.wheelSpeeds[2] = forwardSpeed + strafeSpeed - turnSpeed;
        this.wheelSpeeds[3] = -forwardSpeed + strafeSpeed - turnSpeed;

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

        if(switchMode) {
           this.setState(RobotStates.Drivetrain.HALF_SPEED);
        } else {
            this.setState(RobotStates.Drivetrain.FULL_SPEED);
        }

        if(getDrivetrainState() == RobotStates.Drivetrain.HALF_SPEED) {
            this.motorFL.setPower(this.wheelSpeeds[0]);
            this.motorFR.setPower(this.wheelSpeeds[1]);
            this.motorBL.setPower(this.wheelSpeeds[2]);
            this.motorBR.setPower(this.wheelSpeeds[3]);
        } else {
            this.motorFL.setPower(this.wheelSpeeds[0] * 2);
            this.motorFR.setPower(this.wheelSpeeds[1] * 2);
            this.motorBL.setPower(this.wheelSpeeds[2] * 2);
            this.motorBR.setPower(this.wheelSpeeds[3] * 2);
        }
    }
//    public void drivetrainData(Telemetry telemetry) {
//        telemetry.addData("Front Left: ", this.motorFL.getCurrentPosition() * -1);
//        telemetry.addData("Front Right: ", this.motorFR.getCurrentPosition());
//        telemetry.addData("Back Left: ", this.motorBL.getCurrentPosition() * -1);
//        telemetry.addData("Back Right: ", this.motorBR.getCurrentPosition());
//
//        telemetry.addData("Velocity FL: ", this.motorFL.getVelocity());
//        telemetry.addData("Velocity FR:", this.motorBR.getVelocity());
//        telemetry.addData("Velocity BL: ", this.motorBL.getVelocity());
//        telemetry.addData("Velocity BR: ", this.motorBR.getVelocity());
//
//        telemetry.addData("IMU angle: ", this.imu.getRobotYawPitchRollAngles());
//        telemetry.update();
//    }
}
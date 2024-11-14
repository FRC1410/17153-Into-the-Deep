package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Timer;
import java.util.TimerTask;

@TeleOp
public class Robot extends OpMode {
    Drivetrain drivetrain = new Drivetrain();

    public void init() {
        this.drivetrain.init(hardwareMap);
    }

    @Override
    public void loop() {
        this.drivetrain.drivetrainData(this.telemetry);

        this.drivetrain.mechanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
    }
}
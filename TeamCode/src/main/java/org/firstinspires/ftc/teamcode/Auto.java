package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Timer;
import java.util.TimerTask;

import static org.firstinspires.ftc.teamcode.TimeHelper.*;
import static org.firstinspires.ftc.teamcode.Constants.*;

@Autonomous (name = "Test For acceleration")
public class Auto extends LinearOpMode {
    private final Timer timer = new Timer();
    private final TimerTask timerTask = new TimeHelper();
    private final Drivetrain drivetrain = new Drivetrain();

    private final long timeInMilliseconds = 1000000;

    @Override
    public void runOpMode() throws InterruptedException {
        this.waitForStart();
        this.drivetrain.autoInit(hardwareMap);

        this.telemetry.addLine(TIME + " seconds");

        while(opModeIsActive()) {
            this.timer.schedule(timerTask, 0, timeInMilliseconds);
            this.telemetry.update();

            if(this.drivetrain.getCurrentVelocity() == MAX_VELOCITY) {
                this.timer.cancel();
                this.drivetrain.autoDriveStraight(10000000, 0);
            } else {
                this.drivetrain.autoDriveStraight(10000000, MAX_VELOCITY);
            }
        }
    }
}

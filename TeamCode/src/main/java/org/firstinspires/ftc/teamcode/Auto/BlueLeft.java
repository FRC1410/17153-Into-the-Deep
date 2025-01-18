package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Actions.Lower;
import org.firstinspires.ftc.teamcode.Actions.Raise;
import org.firstinspires.ftc.teamcode.Subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystem.Shoulder;
import org.firstinspires.ftc.teamcode.Subsystem.Wrist;


@Autonomous(name="Robot: Red Left side Auto", group="Auto")

public class BlueLeft extends LinearOpMode {

    private final AutoDriveTrain drivetrain = new AutoDriveTrain();
    private final Shoulder shoulder = new Shoulder();
    private final LinearSlide linearSlide = new LinearSlide();
    private final Wrist wrist = new Wrist();

    private final Raise raiseFullCommand = new Raise(linearSlide, shoulder, wrist);
    private final Lower lowerCommand = new Lower(linearSlide, shoulder, wrist);

    @Override
    public void runOpMode() throws InterruptedException {

        this.drivetrain.init(this.hardwareMap);
        this.shoulder.init(this.hardwareMap);
        this.linearSlide.init(this.hardwareMap);
        this.wrist.init(this.hardwareMap);

        waitForStart();




        drivetrain.drive(0,-1,0);
        Thread.sleep(3000);
        drivetrain.drive(0,0,0);
        Thread.sleep(100);

    }
}
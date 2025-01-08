package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Actions.Lower;
import org.firstinspires.ftc.teamcode.Actions.RaiseFull;
import org.firstinspires.ftc.teamcode.Subsystem.Shoulder;
import org.firstinspires.ftc.teamcode.Subsystem.Claw;
import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystem.Wrist;


@Autonomous(name="Robot: Red Left side Auto", group="Auto")

public class RedLeft extends LinearOpMode {

    private final Drivetrain drivetrain = new Drivetrain();
    private final Shoulder shoulder = new Shoulder();
    private final LinearSlide linearSlide = new LinearSlide();
    private final Claw claw = new Claw();
    private final Wrist wrist = new Wrist();

    private final RaiseFull raiseFullCommand = new RaiseFull(linearSlide, shoulder, wrist);
    private final Lower lowerCommand = new Lower(linearSlide, shoulder, wrist);

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        this.drivetrain.init(this.hardwareMap);
        this.shoulder.init(this.hardwareMap);
        this.linearSlide.init(this.hardwareMap);
        this.claw.init(this.hardwareMap);
        this.wrist.init(this.hardwareMap);



        new RaiseFull(linearSlide, shoulder, wrist).raise();
        Thread.sleep(300);
        new Lower(linearSlide, shoulder, wrist).lower();
        Thread.sleep(300);
        this.drivetrain.mechanumDrive(0,0.5,0,false);
        Thread.sleep(300);

    }
}
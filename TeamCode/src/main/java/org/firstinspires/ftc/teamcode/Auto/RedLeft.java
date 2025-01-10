package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Actions.Lower;
import org.firstinspires.ftc.teamcode.Actions.Raise;
import org.firstinspires.ftc.teamcode.Subsystem.Shoulder;
import org.firstinspires.ftc.teamcode.Subsystem.Claw;
import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystem.Wrist;
import org.firstinspires.ftc.teamcode.Auto.*;
import org.firstinspires.ftc.teamcode.Subsystem.Shoulder;
import org.firstinspires.ftc.teamcode.Util.RobotStates;
import org.firstinspires.ftc.teamcode.Subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystem.Wrist;


@Autonomous(name="Robot: Red Left side Auto", group="Auto")

public class RedLeft extends LinearOpMode {

    private final AutoDriveTrain drivetrain = new AutoDriveTrain();
    private final Shoulder shoulder = new Shoulder();
    private final LinearSlide linearSlide = new LinearSlide();
    private final Claw claw = new Claw();
    private final Wrist wrist = new Wrist();

    private final Raise raiseFullCommand = new Raise(linearSlide, shoulder, wrist);
    private final Lower lowerCommand = new Lower(linearSlide, shoulder, wrist);

    @Override
    public void runOpMode() throws InterruptedException {

        this.drivetrain.init(this.hardwareMap);
        this.shoulder.init(this.hardwareMap);
        this.linearSlide.init(this.hardwareMap);
        this.claw.init(this.hardwareMap);
        this.wrist.init(this.hardwareMap);

        waitForStart();

//        this.shoulder.setState(RobotStates.Arm.UP);
//        Thread.sleep(100);
//        this.linearSlide.setState(RobotStates.LinearSlide.HIGH_SCORE);
//        Thread.sleep(300);
//        this.linearSlide.setState(RobotStates.LinearSlide.START_POS);
//        Thread.sleep(100);
//        this.shoulder.setState(RobotStates.Arm.DOWN);
//        Thread.sleep(300);
//        drivetrain.drive(7,0,0);
//        Thread.sleep(300);
        drivetrain.drive(0,1,0);
        Thread.sleep(1900);
        drivetrain.drive(0,0,0);

    }
}
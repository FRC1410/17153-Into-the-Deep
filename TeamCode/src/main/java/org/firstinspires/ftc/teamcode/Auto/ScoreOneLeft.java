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
import org.firstinspires.ftc.teamcode.Util.RobotStates;


@Autonomous(name="Robot: Score one on left (TESTING ONLY)", group="Auto")

public class ScoreOneLeft extends LinearOpMode {

    private double count = 0;

    private final AutoDriveTrain drivetrain = new AutoDriveTrain();
    private final Shoulder shoulder = new Shoulder();
    private final LinearSlide linearSlide = new LinearSlide();
    private final Claw claw = new Claw();
    private final Wrist wrist = new Wrist();

    private final Raise raiseFullCommand = new Raise(linearSlide, shoulder, wrist);
    private final Lower lowerCommand = new Lower(linearSlide, shoulder, wrist);

    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain.init(this.hardwareMap);
        shoulder.init(this.hardwareMap);
//        linearSlide.init(this.hardwareMap);
//        claw.init(this.hardwareMap);
        wrist.init(this.hardwareMap);

        waitForStart();

        telemetry.addData("Auto started", opModeIsActive());
        telemetry.update();

        Thread.sleep(200);

        drivetrain.drive(0, 1, 0);
        Thread.sleep(1000);
        drivetrain.drive(0, 0, 0);

        this.shoulder.setState(RobotStates.Arm.UP);

        while (!Shoulder.hasReachedState && opModeIsActive()) {
            this.shoulder.goToState();
            telemetry.addData("running?", this.shoulder.getArmState());
            telemetry.update();
        }

        Thread.sleep(1000);

        this.shoulder.setState(RobotStates.Arm.DOWN);

        while (!Shoulder.hasReachedState && opModeIsActive()) {
            this.shoulder.goToState();
            shoulder.armTelemetry(telemetry);
            wrist.wristTelemetry(telemetry);
            telemetry.update();
        }

        Thread.sleep(2000);

    }
}

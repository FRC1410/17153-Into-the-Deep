package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Actions.Lower;
import org.firstinspires.ftc.teamcode.Actions.Raise;
import org.firstinspires.ftc.teamcode.Subsystem.Shoulder;
import org.firstinspires.ftc.teamcode.Subsystem.Claw;
import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystem.Wrist;
import org.firstinspires.ftc.teamcode.Auto.*;
import org.firstinspires.ftc.teamcode.Util.RobotStates;
import org.firstinspires.ftc.teamcode.Auto.AutoWrist;


@Autonomous(name="Robot: Score one on left (TESTING ONLY)", group="Auto")

public class ScoreOneLeft extends LinearOpMode {

    private double count = 0;

    private final AutoDriveTrain drivetrain = new AutoDriveTrain();
    private final AutoShoulder shoulder = new AutoShoulder();
    private final AutoLinearSlide linearSlide = new AutoLinearSlide();
    private final Claw claw = new Claw();
    private final AutoWrist wrist = new AutoWrist();


    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain.init(this.hardwareMap);
        shoulder.init(this.hardwareMap);
        wrist.init(this.hardwareMap);

        linearSlide.init(this.hardwareMap);
//        claw.init(this.hardwareMap);


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
            telemetry.addData("Shoulder State", this.shoulder.getArmState());
            telemetry.update();
        }

        Thread.sleep(1000);
        wrist.setState(RobotStates.Wrist.SCORE);
        while (!AutoWrist.hasReachedState && opModeIsActive()) {
            wrist.goToState();
            wrist.wristTelemetry(telemetry);
            telemetry.update();
        }
         //Linear Slide to HIGH_SCORE
//        linearSlide.setState(RobotStates.LinearSlide.HIGH_SCORE);
//        while (!LinearSlide.hasReachedState && opModeIsActive()) {
//            linearSlide.goToState(0,0); // Ensure this method is non-blocking
//            linearSlide.slideData(telemetry);
//            telemetry.update();
//        }

        // Wait for 5 seconds
        Thread.sleep(5000);

        // Linear Slide to LOW_SCORE
//        linearSlide.setState(RobotStates.LinearSlide.START_POS);
//        while (!LinearSlide.hasReachedState && opModeIsActive()) {
//            linearSlide.goToState(0, 0);
//            linearSlide.slideData(telemetry);
//            telemetry.update();
//        }
//
//
            this.shoulder.setState(RobotStates.Arm.DOWN);
        while (!Shoulder.hasReachedState && opModeIsActive()) {
            this.shoulder.goToState();
            shoulder.armTelemetry(telemetry);
            wrist.wristTelemetry(telemetry);
            telemetry.update();
        }

        Thread.sleep(5000);
    }
}

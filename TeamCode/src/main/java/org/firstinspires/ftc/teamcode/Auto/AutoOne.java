package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Subsystem.Claw.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystem.Claw;
import org.firstinspires.ftc.teamcode.Util.RobotStates;


@Autonomous(name="Robot: Auto Concept 1", group="Auto")
public class AutoOne extends LinearOpMode {

    private final AutoDriveTrain drivetrain = new AutoDriveTrain();
    private final AutoShoulder shoulder = new AutoShoulder();
    private final AutoLinearSlide linearSlide = new AutoLinearSlide();
    private final Claw claw = new Claw();
    private final AutoWrist wrist = new AutoWrist();
    long startTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain.init(this.hardwareMap);
        shoulder.init(this.hardwareMap);
        wrist.init(this.hardwareMap);
        linearSlide.init(this.hardwareMap);
        claw.init(this.hardwareMap);

        waitForStart();

        telemetry.addData("Auto started", opModeIsActive());
        telemetry.update();


        // Initial drivetrain move
        drivetrain.drive(0, -1, 0);
        Thread.sleep(400);
        drivetrain.drive(0, 0, 0);

        drivetrain.drive(1, 0, 0);
        Thread.sleep(650);
        drivetrain.drive(0, 0, 0);

        drivetrain.drive(0, 0, 1);
        Thread.sleep(550);
        drivetrain.drive(0, 0, 0);


//        this.shoulder.setState(RobotStates.Arm.UP);
//        wrist.setState(RobotStates.Wrist.FLOOR);
//        while ((shoulder.getEncoderVal() < 1650 || shoulder.getEncoderVal() > 1900) && opModeIsActive()) {
//            this.shoulder.goToState();
//            wrist.goToState();
//            shoulder.armTelemetry(telemetry);
//            wrist.wristTelemetry(telemetry);
//            claw.clawTelemetry(telemetry);
//            linearSlide.slideData(telemetry);
//            telemetry.update();
//        }

//        drivetrain.drive(1, 0, 0);
//        Thread.sleep(250);
//        drivetrain.drive(0, 0, 0);
//        Thread.sleep(1000);

//        linearSlide.setState(RobotStates.LinearSlide.LOW_SCORE);
//        wrist.setState(RobotStates.Wrist.FLOOR);
//        while ((linearSlide.getLeftEncoderVal() < 1_170 || linearSlide.getLeftEncoderVal() > 1_250) && (linearSlide.getRightEncoderVal() < 1_170 || linearSlide.getRightEncoderVal() > 1_250) && opModeIsActive()) {
//            linearSlide.goToState(0, 0);
//            wrist.goToState();
//            shoulder.armTelemetry(telemetry);
//            wrist.wristTelemetry(telemetry);
//            claw.clawTelemetry(telemetry);
//            linearSlide.slideData(telemetry);
//            telemetry.update();
//        }

//        Thread.sleep(1000);
//        drivetrain.drive(0, 0, -1);
//        Thread.sleep(350);
//        drivetrain.drive(0, 0, 0);


//        wrist.setState(RobotStates.Wrist.SCORE);
//        while (!AutoWrist.hasReachedState && opModeIsActive()) {
//            wrist.goToState();
//            shoulder.armTelemetry(telemetry);
//            wrist.wristTelemetry(telemetry);
//            claw.clawTelemetry(telemetry);
//            linearSlide.slideData(telemetry);
//            telemetry.update();
//        }
        Thread.sleep(500);


        claw.setClawState(RobotStates.Claw.OPEN);
        while (!hasReachedState && opModeIsActive()) {
            claw.goToState();
            wrist.goToState();
            shoulder.armTelemetry(telemetry);
            wrist.wristTelemetry(telemetry);
            claw.clawTelemetry(telemetry);
            linearSlide.slideData(telemetry);
            telemetry.update();
        }

        Thread.sleep(500);
//
//        drivetrain.drive(-1,0,0);
//        Thread.sleep(10);
//        drivetrain.drive(0,0,0);

//        wrist.setState(RobotStates.Wrist.FLOOR);
//        while (!AutoWrist.hasReachedState && opModeIsActive()) {
//            wrist.goToState();
//            wrist.goToState();
//            shoulder.armTelemetry(telemetry);
//            wrist.wristTelemetry(telemetry);
//            claw.clawTelemetry(telemetry);
//            linearSlide.slideData(telemetry);
//            telemetry.update();
//        }


//        linearSlide.setState(RobotStates.LinearSlide.START_POS);
//        wrist.setState(RobotStates.Wrist.FLOOR);
//        while ((linearSlide.getLeftEncoderVal() > 30 || linearSlide.getLeftEncoderVal() < -30) && (linearSlide.getRightEncoderVal() > 30 || linearSlide.getRightEncoderVal() < -30) && opModeIsActive()) {
//            linearSlide.goToState(0, 0);
//            wrist.goToState();
//            shoulder.armTelemetry(telemetry);
//            wrist.wristTelemetry(telemetry);
//            claw.clawTelemetry(telemetry);
//            linearSlide.slideData(telemetry);
//            telemetry.update();
//        }

//        this.shoulder.setState(RobotStates.Arm.DOWN);
//        wrist.setState(RobotStates.Wrist.FLOOR);
//        while ((shoulder.getEncoderVal() < 15 || shoulder.getEncoderVal() > -15) && opModeIsActive()) {
//            this.shoulder.goToState();
//            wrist.goToState();
//            shoulder.armTelemetry(telemetry);
//            wrist.wristTelemetry(telemetry);
//            claw.clawTelemetry(telemetry);
//            linearSlide.slideData(telemetry);
//            telemetry.update();
//        }
//        drivetrain.drive(-1, 0, 0);
//        Thread.sleep(320);
//        drivetrain.drive(0, 0, 0);
//        drivetrain.drive(0, 0, 1);
//        Thread.sleep(200);
//        drivetrain.drive(0, 0, 0);


        claw.setClawState(RobotStates.Claw.CLOSED);
        while (!hasReachedState && opModeIsActive()) {
            claw.goToState();
            wrist.goToState();
            shoulder.armTelemetry(telemetry);
            wrist.wristTelemetry(telemetry);
            claw.clawTelemetry(telemetry);
            linearSlide.slideData(telemetry);
            telemetry.update();
        }

    }
}

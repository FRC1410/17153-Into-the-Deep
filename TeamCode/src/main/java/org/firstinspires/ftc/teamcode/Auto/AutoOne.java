package org.firstinspires.ftc.teamcode.Auto;

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

        drivetrain.drive(0, 0, -1);
        Thread.sleep(350);
        drivetrain.drive(0, 0, 0);


        this.shoulder.setState(RobotStates.Arm.UP);
        while ((shoulder.getEncoderVal() < 1650 || shoulder.getEncoderVal() > 1900) && opModeIsActive()) {
            this.shoulder.goToState();
            shoulder.armTelemetry(telemetry);
            wrist.wristTelemetry(telemetry);
            claw.clawTelemetry(telemetry);
            linearSlide.slideData(telemetry);
            telemetry.update();
        }

        drivetrain.drive(1, 0, 0);
        Thread.sleep(250);
        drivetrain.drive(0, 0, 0);

        linearSlide.setState(RobotStates.LinearSlide.LOW_SCORE);
        while ((linearSlide.getLeftEncoderVal() > 580 || linearSlide.getLeftEncoderVal() < 650) && (linearSlide.getRightEncoderVal() > 580 || linearSlide.getRightEncoderVal() < 650) && opModeIsActive()) {
            linearSlide.goToState(0, 0);
            shoulder.armTelemetry(telemetry);
            wrist.wristTelemetry(telemetry);
            claw.clawTelemetry(telemetry);
            linearSlide.slideData(telemetry);
            telemetry.update();
        }

        Thread.sleep(2000);
        drivetrain.drive(0,0,1);
        Thread.sleep(1000);
        drivetrain.drive(0,0,0);

        wrist.setState(RobotStates.Wrist.SCORE);
        while (!AutoWrist.hasReachedState && opModeIsActive()) {
            wrist.goToState();
            shoulder.armTelemetry(telemetry);
            wrist.wristTelemetry(telemetry);
            claw.clawTelemetry(telemetry);
            linearSlide.slideData(telemetry);
            telemetry.update();
        }
        Thread.sleep(5000);

        claw.setClawState(RobotStates.Claw.OPEN);
        startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < 1000 && opModeIsActive()) {
            claw.goToState();
            shoulder.armTelemetry(telemetry);
            wrist.wristTelemetry(telemetry);
            claw.clawTelemetry(telemetry);
            linearSlide.slideData(telemetry);
            telemetry.update();
        }

        wrist.setState(RobotStates.Wrist.FLOOR);
        while (wrist.getCompletion() && opModeIsActive()) {
            wrist.goToState();
            shoulder.armTelemetry(telemetry);
            wrist.wristTelemetry(telemetry);
            claw.clawTelemetry(telemetry);
            linearSlide.slideData(telemetry);
            telemetry.update();
        }

        linearSlide.setState(RobotStates.LinearSlide.START_POS);
        while ((linearSlide.getLeftEncoderVal() > 15 || linearSlide.getLeftEncoderVal() < -15) && (linearSlide.getRightEncoderVal() > 15 || linearSlide.getRightEncoderVal() < -15) && opModeIsActive()) {
            linearSlide.goToState(0, 0);
            shoulder.armTelemetry(telemetry);
            wrist.wristTelemetry(telemetry);
            claw.clawTelemetry(telemetry);
            linearSlide.slideData(telemetry);
            telemetry.update();
        }

    }
}
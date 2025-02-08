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
    double i = 0;
    double j = 0;
    double k = 0;
    double l = 0;

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
        Thread.sleep(430);
        drivetrain.drive(0, 0, 0);

        drivetrain.drive(1, 0, 0);
        Thread.sleep(680);
        drivetrain.drive(0, 0, 0);

        drivetrain.drive(0, 0, -1);
        Thread.sleep(400);
        drivetrain.drive(0, 0, 0);
//
//


        this.shoulder.setState(RobotStates.Arm.UP);
        wrist.setState(RobotStates.Wrist.FLOOR);
        while ((shoulder.getEncoderVal() < 1650 || shoulder.getEncoderVal() > 1900) && opModeIsActive()) {
            this.shoulder.goToState();
            wrist.goToState();
            shoulder.armTelemetry(telemetry);
            wrist.wristTelemetry(telemetry);
            claw.clawTelemetry(telemetry);
            linearSlide.slideData(telemetry);
            telemetry.update();
        }



        linearSlide.setState(RobotStates.LinearSlide.LOW_SCORE);
        wrist.setState(RobotStates.Wrist.FLOOR);
        while ((linearSlide.getLeftEncoderVal() < 2_720 || linearSlide.getLeftEncoderVal() > 2_780) && (linearSlide.getRightEncoderVal() < 2_720 || linearSlide.getRightEncoderVal() > 2_780) && opModeIsActive()) {
            linearSlide.goToState(0, 0);
            wrist.goToState();
            shoulder.armTelemetry(telemetry);
            wrist.wristTelemetry(telemetry);
            claw.clawTelemetry(telemetry);
            linearSlide.slideData(telemetry);
            telemetry.update();
        }



        wrist.setState(RobotStates.Wrist.SCORE);
        while (!AutoWrist.hasReachedState && opModeIsActive()) {
            wrist.goToState();
            shoulder.armTelemetry(telemetry);
            wrist.wristTelemetry(telemetry);
            claw.clawTelemetry(telemetry);
            linearSlide.slideData(telemetry);
            telemetry.update();
        }

        Thread.sleep(500);
        drivetrain.drive(1, 0, 0);
        Thread.sleep(300);
        drivetrain.drive(0, 0, 0);
        Thread.sleep(1000);



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
        drivetrain.drive(-1,0,0);
        Thread.sleep(110);
        drivetrain.drive(0,0,0);

        wrist.setState(RobotStates.Wrist.FLOOR);
        while (!AutoWrist.hasReachedState && opModeIsActive()) {
            wrist.goToState();
            wrist.goToState();
            i ++;
            shoulder.armTelemetry(telemetry);
            wrist.wristTelemetry(telemetry);
            claw.clawTelemetry(telemetry);
            linearSlide.slideData(telemetry);
            telemetry.addData("Count I", i);
            telemetry.addData("Count J", j);
            telemetry.addData("Count K", k);
            telemetry.addData("Count L", l);
            telemetry.update();
        }

        drivetrain.drive(-1,0,0);
        Thread.sleep(120);
        drivetrain.drive(0,0,0);


        linearSlide.setState(RobotStates.LinearSlide.START_POS);
        wrist.setState(RobotStates.Wrist.FLOOR);
        while ((linearSlide.getLeftEncoderVal() > 30 || linearSlide.getLeftEncoderVal() < -35) && (linearSlide.getRightEncoderVal() > 30 || linearSlide.getRightEncoderVal() < -35) && opModeIsActive()) {
            linearSlide.goToState(0, 0);
            wrist.goToState();
            j ++;
            shoulder.armTelemetry(telemetry);
            wrist.wristTelemetry(telemetry);
            claw.clawTelemetry(telemetry);
            linearSlide.slideData(telemetry);
            telemetry.addData("Count I", i);
            telemetry.addData("Count J", j);
            telemetry.addData("Count K", k);
            telemetry.addData("Count L", l);
            telemetry.update();

        }

        this.shoulder.setState(RobotStates.Arm.DOWN);
        wrist.setState(RobotStates.Wrist.FLOOR);
        while ((shoulder.getEncoderVal() < 35 && shoulder.getEncoderVal() > -35) && opModeIsActive()) {
            this.shoulder.goToState();
            wrist.goToState();
            k ++;
            shoulder.armTelemetry(telemetry);
            wrist.wristTelemetry(telemetry);
            claw.clawTelemetry(telemetry);
            linearSlide.slideData(telemetry);
            telemetry.addData("Count I", i);
            telemetry.addData("Count J", j);
            telemetry.addData("Count K", k);
            telemetry.addData("Count L", l);
            telemetry.update();
        }

//        drivetrain.drive(-1, 0, 0);
//        Thread.sleep(320);
//        drivetrain.drive(0, 0, 0);
//        drivetrain.drive(0, 0, 1);
//        Thread.sleep(200);
//        drivetrain.drive(0, 0, 0);
        drivetrain.drive(0,0,-1);
        Thread.sleep(360);
        drivetrain.drive(0,0,0);

        drivetrain.drive(-1,0,0);
        Thread.sleep(318);
        drivetrain.drive(0,0,0);


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

package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystem.Claw;
import org.firstinspires.ftc.teamcode.Util.RobotStates;

@Autonomous(name="Robot: Autonomous Testing", group="Auto")
public class AutoTest extends LinearOpMode {

    private final AutoDriveTrain drivetrain = new AutoDriveTrain();
    private final AutoShoulder shoulder = new AutoShoulder();
    private final AutoLinearSlide linearSlide = new AutoLinearSlide();
    private final Claw claw = new Claw();
    private final AutoWrist wrist = new AutoWrist();
    int z = 1;

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

//        this.shoulder.setState(RobotStates.Arm.UP);
//        while ((shoulder.getEncoderVal() < 1650 || shoulder.getEncoderVal() > 1900) && opModeIsActive()) {
//            this.shoulder.goToState();
//            shoulder.armTelemetry(telemetry);
//            wrist.wristTelemetry(telemetry);
//            claw.clawTelemetry(telemetry);
//            linearSlide.slideData(telemetry);
//            telemetry.update();
//        }
//
////        // Initial drivetrain move
//        drivetrain.drive(0, 0, 1);
//        Thread.sleep(1000);
//        drivetrain.drive(0, 0, 0);

        // Move the shoulder to the UP state for 5 seconds
//        this.shoulder.setState(RobotStates.Arm.UP);
//        long startTime = System.currentTimeMillis();
//        while (System.currentTimeMillis() - startTime < 5000 && opModeIsActive()) {
//            this.shoulder.goToState();
//            shoulder.armTelemetry(telemetry);
//            wrist.wristTelemetry(telemetry);
//            claw.clawTelemetry(telemetry);
//            linearSlide.slideData(telemetry);
//            telemetry.update();
//        }
//
//        // Move the wrist to the SCORE state for 5 seconds
//        wrist.setState(RobotStates.Wrist.SCORE);
//        startTime = System.currentTimeMillis();
//        while (System.currentTimeMillis() - startTime < 5000 && opModeIsActive()) {
//            wrist.goToState();
//            shoulder.armTelemetry(telemetry);
//            wrist.wristTelemetry(telemetry);
//            claw.clawTelemetry(telemetry);
//            linearSlide.slideData(telemetry);
//            telemetry.update();
//        }
//        claw.setClawState(RobotStates.Claw.OPEN);
//        startTime = System.currentTimeMillis();
//        while (System.currentTimeMillis() - startTime < 5000 && opModeIsActive()) {
//            claw.goToState();
//            shoulder.armTelemetry(telemetry);
//            wrist.wristTelemetry(telemetry);
//            claw.clawTelemetry(telemetry);
//            linearSlide.slideData(telemetry);
//            telemetry.update();
//        }
//
//        // Wait for 5 seconds
//        Thread.sleep(5000);
        linearSlide.setState(RobotStates.LinearSlide.MANUEL);
//        wrist.setState(RobotStates.Wrist.FLOOR);
        while ((linearSlide.getLeftEncoderVal() < 550 || linearSlide.getLeftEncoderVal() > 480) && (linearSlide.getRightEncoderVal() < 550 || linearSlide.getRightEncoderVal() > 480) && opModeIsActive()) {
            linearSlide.goToState((z-1), z);
//            wrist.goToState();
            z += 1;
            shoulder.armTelemetry(telemetry);
            wrist.wristTelemetry(telemetry);
            claw.clawTelemetry(telemetry);
            linearSlide.slideData(telemetry);
            telemetry.update();
        }

//        // Move the linear slide to the START_POS state for 5 seconds
//        linearSlide.setState(RobotStates.LinearSlide.LOW_SCORE);
//        startTime = System.currentTimeMillis();
//        while (System.currentTimeMillis() - startTime < 5000 && opModeIsActive()) {
//            linearSlide.goToState(0, 0);
//            shoulder.armTelemetry(telemetry);
//            wrist.wristTelemetry(telemetry);
//            claw.clawTelemetry(telemetry);
//            linearSlide.slideData(telemetry);
//            telemetry.update();
//        }
//        Thread.sleep(5000);
//        wrist.setState(RobotStates.Wrist.FLOOR);
//        startTime = System.currentTimeMillis();
//        while (System.currentTimeMillis() - startTime < 5000 && opModeIsActive()) {
//            wrist.goToState();
//            shoulder.armTelemetry(telemetry);
//            wrist.wristTelemetry(telemetry);
//            claw.clawTelemetry(telemetry);
//            linearSlide.slideData(telemetry);
//            telemetry.update();
//        }
//        linearSlide.setState(RobotStates.LinearSlide.START_POS);
//        startTime = System.currentTimeMillis();
//        while (System.currentTimeMillis() - startTime < 5000 && opModeIsActive()) {
//            linearSlide.goToState(0, 0);
//            shoulder.armTelemetry(telemetry);
//            wrist.wristTelemetry(telemetry);
//            claw.clawTelemetry(telemetry);
//            linearSlide.slideData(telemetry);
//            telemetry.update();
//        }
//
//        // Move the shoulder to the DOWN state for 5 seconds
//        this.shoulder.setState(RobotStates.Arm.DOWN);
//        startTime = System.currentTimeMillis();
//        while (System.currentTimeMillis() - startTime < 5000 && opModeIsActive()) {
//            this.shoulder.goToState();
//            shoulder.armTelemetry(telemetry);
//            wrist.wristTelemetry(telemetry);
//            claw.clawTelemetry(telemetry);
//            linearSlide.slideData(telemetry);
//            telemetry.update();
//        }

        // Wait for 5 seconds before ending the OpMode
//        Thread.sleep(5000);
    }
}

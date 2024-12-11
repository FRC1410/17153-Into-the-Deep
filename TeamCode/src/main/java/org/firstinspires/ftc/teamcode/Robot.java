package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Actions.LiftWrist;
import org.firstinspires.ftc.teamcode.Actions.Lower;
import org.firstinspires.ftc.teamcode.Actions.Raise;
import org.firstinspires.ftc.teamcode.Subsystem.Arm;
import org.firstinspires.ftc.teamcode.Subsystem.Claw;
import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystem.Wrist;
import org.firstinspires.ftc.teamcode.Util.RobotStates;
import org.firstinspires.ftc.teamcode.Util.Toggle;

@TeleOp
public class Robot extends OpMode {
    private final Drivetrain drivetrain = new Drivetrain();
    private final Arm arm = new Arm();
    private final LinearSlide linearSlide = new LinearSlide();
    private final Claw claw = new Claw();
    private final Wrist wrist = new Wrist();

    private final Raise raiseCommand = new Raise(linearSlide, arm, wrist);
    private final Lower lowerCommand = new Lower(linearSlide, arm, wrist);
    private final LiftWrist liftWristCommand = new LiftWrist(wrist);

    private final Toggle raiseToggle = new Toggle();
    private final Toggle clawToggle = new Toggle();
//    private final Toggle wristToggle = new Toggle();
    
    public void init() {
        this.drivetrain.init(hardwareMap);
        this.arm.init(hardwareMap);
        this.linearSlide.init(hardwareMap);
        this.claw.init(hardwareMap);
        this.wrist.init(hardwareMap);
    }

    @Override
    public void loop() {
        this.linearSlide.slideData(telemetry);
        this.arm.armTelemetry(telemetry);
        this.wrist.wristTelemetry(telemetry);

        if(raiseToggle.toggleButton(gamepad2.y)) {
            if(wrist.getCurrentState() != RobotStates.Wrist.SCORE) {
                this.raiseCommand.raise();
            }
        } else {
            if(this.arm.getArmState() != RobotStates.Arm.DOWN) {
                this.lowerCommand.lower();
                this.linearSlide.setState(RobotStates.LinearSlide.MANUEL);
            }
        }

        if(this.clawToggle.toggleButton(gamepad2.right_bumper)) {
            this.claw.setClawState(RobotStates.Claw.CLOSED);
        } else {
            this.claw.setClawState(RobotStates.Claw.OPEN);
        }

//        if(this.wristToggle.toggleButton(gamepad2.left_bumper)) {
//            this.wrist.setState(RobotStates.Wrist.SCORE);
//        } else {
//            this.wrist.setState(RobotStates.Wrist.FLOOR);
//        }

        if(gamepad2.left_bumper) {
            this.liftWristCommand.liftWrist();
        }

        this.drivetrain.mechanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        this.linearSlide.goToState((int) gamepad2.right_trigger, (int) gamepad2.left_trigger);
        this.arm.goToState();

        this.wrist.goToState();
        this.claw.goToState();
    }
}
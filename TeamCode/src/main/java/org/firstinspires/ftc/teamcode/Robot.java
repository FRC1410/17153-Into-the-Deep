package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    private final Toggle raiseToggle = new Toggle();
    private final Toggle clawToggle = new Toggle();
    private final Toggle wristToggle = new Toggle();
    
    public void init() {
        this.drivetrain.init(this.hardwareMap);
        this.arm.init(this.hardwareMap);
        this.linearSlide.init(this.hardwareMap);
        this.claw.init(this.hardwareMap);
        this.wrist.init(this.hardwareMap);
    }

    @Override
    public void loop() {
        this.linearSlide.slideData(this.telemetry);
        this.arm.armTelemetry(this.telemetry);
        this.wrist.wristTelemetry(this.telemetry);

        if(raiseToggle.toggleButton(gamepad2.y)) {
            new Raise(linearSlide, arm, wrist).raise();
        } else {
            new Lower(linearSlide, arm, wrist).lower();
            this.linearSlide.setState(RobotStates.LinearSlide.MANUEL);
        }

//        if(this.gamepad2.y) {
//            new Raise(linearSlide, arm, wrist).raise();
//        }
//
//        if(this.gamepad2.right_bumper) {
//            new Lower(linearSlide, arm, wrist).lower();
//            this.linearSlide.setState(RobotStates.LinearSlide.MANUEL);
//        }

        if(this.clawToggle.toggleButton(this.gamepad1.right_bumper)) {
            this.claw.setClawState(RobotStates.Claw.CLOSED);
        } else {
            this.claw.setClawState(RobotStates.Claw.OPEN);
        }


        if(wristToggle.toggleButton(this.gamepad2.left_bumper)) {
            this.wrist.setState(RobotStates.Wrist.SCORE);
        } else {
            this.wrist.setState(RobotStates.Wrist.FLOOR);
        }

        this.drivetrain.mechanumDrive(
                this.gamepad1.left_stick_x,
                this.gamepad1.left_stick_y,
                this.gamepad1.right_stick_x,
                this.gamepad1.a
        );

        this.linearSlide.goToState(
                (int) this.gamepad2.right_trigger,
                (int) this.gamepad2.left_trigger
        );
        this.arm.goToState();

        this.wrist.goToState();
        this.claw.goToState();
    }
}
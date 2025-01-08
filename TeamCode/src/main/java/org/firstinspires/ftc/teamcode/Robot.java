package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Actions.DoClimb;
import org.firstinspires.ftc.teamcode.Actions.InitClimb;
import org.firstinspires.ftc.teamcode.Actions.Lower;
import org.firstinspires.ftc.teamcode.Actions.RaiseFull;
import org.firstinspires.ftc.teamcode.Subsystem.Shoulder;
import org.firstinspires.ftc.teamcode.Subsystem.Claw;
import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystem.Wrist;
import org.firstinspires.ftc.teamcode.Util.RobotStates;
import org.firstinspires.ftc.teamcode.Util.Toggle;

@TeleOp
public class Robot extends OpMode {
    private final Drivetrain drivetrain = new Drivetrain();
    private final Shoulder shoulder = new Shoulder();
    private final LinearSlide linearSlide = new LinearSlide();
    private final Claw claw = new Claw();
    private final Wrist wrist = new Wrist();

    private final RaiseFull raiseFullCommand = new RaiseFull(linearSlide, shoulder, wrist);
    private final Lower lowerCommand = new Lower(linearSlide, shoulder, wrist);
//    private final InitClimb initClimbCommand = new InitClimb(linearSlide, arm, wrist);
//    private final DoClimb doClimbCommand = new DoClimb(linearSlide, arm, wrist);

    private final Toggle raiseToggle = new Toggle();
    private final Toggle clawToggle = new Toggle();
    private final Toggle wristToggle = new Toggle();
    private final Toggle climbToggle = new Toggle();
    
    public void init() {
        this.drivetrain.init(this.hardwareMap);
        this.shoulder.init(this.hardwareMap);
        this.linearSlide.init(this.hardwareMap);
        this.claw.init(this.hardwareMap);
        this.wrist.init(this.hardwareMap);
    }

    @Override
    public void loop() {
        this.linearSlide.slideData(this.telemetry);
        this.shoulder.armTelemetry(this.telemetry);
        this.wrist.wristTelemetry(this.telemetry);
        this.claw.clawTelemetry(this.telemetry);

        if(raiseToggle.toggleButton(gamepad2.y)) {
            new RaiseFull(linearSlide, shoulder, wrist).raise();
        } else {
            new Lower(linearSlide, shoulder, wrist).lower();
            this.linearSlide.setState(RobotStates.LinearSlide.MANUEL);
            if(climbToggle.toggleButton(gamepad2.a)) {
                new InitClimb(linearSlide, shoulder).climb();
            }
        }

        if(this.gamepad2.x) {
            new DoClimb(linearSlide, shoulder);
        }

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

//        if(climbToggle.toggleButton(gamepad2.a)) {
//            new InitClimb(linearSlide, arm).climb();
//        } else {
//            new DoClimb(linearSlide, arm).climb();
//        }

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
        this.shoulder.goToState();

        this.wrist.goToState();
        this.claw.goToState();
    }
}
package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;



public class Drivetrain {
    DcMotor moaterFL;
    DcMotor moaterFR;
    DcMotor moaterBL;
    DcMotor moaterBR;

    public void init(){
        moaterFL = hardwareMap.dcMotor.get("motorFL");
        moaterFR = hardwareMap.dcMotor.get("motorFR");
        moaterBL = hardwareMap.dcMotor.get("motorBL");
        moaterBR = hardwareMap.dcMotor.get("motorBR");
        moaterFL.setDirection(FORWARD);
        moaterFR.setDirection(FORWARD);
        moaterBL.setDirection(FORWARD);
        moaterBR.setDirection(FORWARD);

    }
    public void loop(){
        moaterFL.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x);
        moaterFR.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x);
        moaterBL.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x);
        moaterBR.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x);

    }

}
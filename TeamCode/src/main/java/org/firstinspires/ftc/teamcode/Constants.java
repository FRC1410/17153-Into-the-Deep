package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class Constants {

    // Robot specs
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
    public static RevHubOrientationOnRobot HUB_ORIENTATION =
            new RevHubOrientationOnRobot(LOGO_FACING_DIRECTION, USB_FACING_DIRECTION);

    public static double GEAR_RATIO = 1 / 18.88;
    public static double WHEEL_RADIUS = 1.5; // In inches
    public static double WHEEL_CIRCUFERENCE = 2 * Math.PI * WHEEL_RADIUS; // In inches
    public static double TRACK_WIDTH = 15.238; // In inches
    public static double ROBOT_WEIGHT = 20; // In pounds

    // Drive motor Constants
    public static double TICKS_PER_REVOLUTION = 28; // Encoder count per revolution
    public static double ENCODER_COUNT_PER_WHEEL_REVOLUTION = TICKS_PER_REVOLUTION * GEAR_RATIO;
    public static double ENCODER_COUNT_PER_INCH = (TICKS_PER_REVOLUTION * GEAR_RATIO) / WHEEL_CIRCUFERENCE;
    public static double MAX_RPM = 6000; // Revolutions per minute
    public static double MAX_MOTOR_WHEEL_VELOCITY =
            (MAX_RPM / 60) * GEAR_RATIO * WHEEL_CIRCUFERENCE; // In inches per second

    public static double MAX_VELOCITY = 50; // Inches per second
    public static double MAX_ACCELERATION = 0;
    public static double MAX_ANGULAR_VELOCITY = 0;

    // Drivetrain PID constants
    public static double DRIVETRAIN_P = 1;
    public static double DRIVETRAIN_I = 0;
    public static double DRIVETRAIN_D = 0;
}

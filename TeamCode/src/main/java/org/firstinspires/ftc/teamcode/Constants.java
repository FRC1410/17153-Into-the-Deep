package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class Constants {
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
    public static RevHubOrientationOnRobot HUB_ORIENTATION =
            new RevHubOrientationOnRobot(LOGO_FACING_DIRECTION, USB_FACING_DIRECTION);

    public static double MAX_VELOCITY = 0;
    public static double MAX_ACCELERATION = 0;
    public static double MAX_ANGULAR_VELOCITY = 0;
    public static double TRACK_WIDTH = 0;

    // Drivetrain PID constants
    public static double DRIVETRAIN_P = 1;
    public static double DRIVETRAIN_I = 0;
    public static double DRIVETRAIN_D = 0;
}

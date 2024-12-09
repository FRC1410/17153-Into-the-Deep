package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "VectorBasedAutonomous", group = "Linear Opmode")
public class VectorBasedAutonomous extends LinearOpMode {

    // Declare drivetrain variable
    private Drivetrain drivetrain;

    @Override
    public void runOpMode() {
        // Initialize the drivetrain
        drivetrain = new Drivetrain();

        // Send telemetry message to signify robot is ready
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Autonomous movement logic
        drivetrain.mechanumDrive(0.5, 0.0, 0.0);  // Strafe right at 50% speed
        sleep(1000);                          // Move for 1 second

        drivetrain.mechanumDrive(0.0, 0.5, 0.0);  // Move forward at 50% speed
        sleep(1000);                          // Move for 1 second

        drivetrain.mechanumDrive(0.0, 0.0, 0.5);  // Turn clockwise at 50% speed
        sleep(1000);                          // Turn for 1 second

        // Stop all motion
        drivetrain.mechanumDrive(0.0, 0.0, 0.0);

        // End of autonomous
        telemetry.addData("Status", "Autonomous Complete");
        telemetry.update();
    }
}

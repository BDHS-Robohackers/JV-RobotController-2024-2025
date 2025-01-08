package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name="Park Left", group="Autonomous")
public class ParkLeftAuto extends LinearOpMode {

    private ElapsedTime time = new ElapsedTime();
    private Robot robot;

    private float FORWARD_TIME = AutonomousConfig.FORWARD_TIME;
    private float FORWARD_SPEED = AutonomousConfig.FORWARD_SPEED;

    private float STRAFE_TIME = AutonomousConfig.STRAFE_TIME;
    private float STRAFE_SPEED = AutonomousConfig.STRAFE_SPEED;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the motors and anything else
        time = new ElapsedTime();
        robot = new Robot();
        robot.initialize(hardwareMap);

        telemetry.addData("Autonomous", "The robot will perform these actions:");
        telemetry.addData("- Idle", "1 second");
        telemetry.addData("- Move forward", FORWARD_TIME+" seconds");
        telemetry.addData("at power", FORWARD_SPEED);
        telemetry.addData("- Move left", STRAFE_TIME+" seconds");
        telemetry.addData("at power", STRAFE_SPEED);
        telemetry.addData("- Move backward", FORWARD_TIME+" seconds");
        telemetry.addData("at power", FORWARD_SPEED);
        telemetry.update();

        waitForStart();

        time.reset();
        while (opModeIsActive()) {
            if (time.seconds() < 1) {
                // Idle
                robot.updateDriveMotors(0, 0, 0);
            } else if (time.seconds() < 1 + FORWARD_TIME) {
                // Move forward a smidge
                robot.updateDriveMotors(FORWARD_SPEED, 0, 0);
            } else if (time.seconds() < 1 + FORWARD_TIME + STRAFE_TIME) {
                // Strafe the specified direction
                robot.updateDriveMotors(0, STRAFE_SPEED, 0);
            } else if (AutonomousConfig.MOVE_BACKWARDS && time.seconds() < 1 + FORWARD_TIME + STRAFE_TIME + FORWARD_SPEED) {
                // Go backwards a smidge
                robot.updateDriveMotors(-FORWARD_SPEED, 0, 0);
            } else {
                // Stop the robot
                robot.updateDriveMotors(0, 0, 0);
            }
        }
    }
}

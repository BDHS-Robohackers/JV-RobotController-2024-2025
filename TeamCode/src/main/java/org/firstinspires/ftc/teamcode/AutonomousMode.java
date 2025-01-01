package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autonomous", group="Autonomous")
public class AutonomousMode extends LinearOpMode {

    ElapsedTime time;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the motors and anything else
        time = new ElapsedTime();

        waitForStart();

        time.reset();
        while (opModeIsActive()) {
            // Run the autonomous mode
            if (time.seconds() < 1) {
                // Make the robot do this
            } else if (time.seconds() < 2){
                // Make the robot do this
            }
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Driver Op Mode", group = "Driver Op Mode")
public class BasicOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Robot robot;

    private int armMotorPosition = 0;
    public static final double INTAKE_POWER = .5;
    public static final double WRIST_ANGLE_PER_SECOND = 40;
    public static final double EXTENSION_POWER = 1;

    Gamepad driverController = gamepad1;
    Gamepad armController = gamepad2;

    @Override
    public void runOpMode() {
        robot = new Robot();
        robot.initialize(hardwareMap);

        // Wait for the robot to start (driver presses PLAY).
        telemetry.addData("Status", "Initialized.");
        telemetry.update();

        waitForStart();
        runtime.reset();

        ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        // Run until the end of the match (driver presses STOP or time runs out).
        while (opModeIsActive()) {
            // Delta time is the amount of time elapsed since the last update.
            double deltaTime = elapsedTime.time();
            elapsedTime.reset();

            updateArm();
            updateWrist(deltaTime);
            updateIntake();
            updateArmExtension();
            updateTeleOpDrive();
        }
    }

    public void updateArm() {
        double armMovement = armController.right_stick_y;

        if (Math.abs(armMovement) <= 0.1) {
            // if joystick is under threshold or not pushed
            robot.armMotor.setPower(1);
            robot.armMotor.setTargetPosition(armMotorPosition);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            // when pressed
            robot.armMotor.setPower(armMovement);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotorPosition = robot.armMotor.getCurrentPosition();
        }
    }

    public void updateWrist(double deltaTime) {
        if (armController.dpad_up) {
            robot.wristServo.rotateByAngle(WRIST_ANGLE_PER_SECOND * deltaTime);
        }
        if (armController.dpad_down) {
            robot.wristServo.rotateByAngle(-WRIST_ANGLE_PER_SECOND * deltaTime);
        }
    }

    public void updateIntake() {
        if (armController.x) {
            robot.intakeMotor.setPower(INTAKE_POWER);
        } else if (armController.b) {
            robot.intakeMotor.setPower(-INTAKE_POWER);
        } else {
            robot.intakeMotor.setPower(0);
        }
    }

    public void updateArmExtension() {
        if (armController.dpad_right) {
            robot.armExtentionMotor.setPower(EXTENSION_POWER);
        } else if (armController.dpad_left) {
            robot.armExtentionMotor.setPower(-EXTENSION_POWER);
        } else {
            robot.armExtentionMotor.setPower(0);
        }
    }

    public void updateTeleOpDrive() {
        double strafingSpeed = gamepad1.left_trigger - gamepad1.right_trigger;
        double axial = (1.0 * gamepad1.left_stick_y); // Forward Back
        double lateral = (0.6 * strafingSpeed); // Strafing (left right)
        double yaw = (0.6 * gamepad1.right_stick_x); // Rotate
        robot.updateDriveMotors(axial, lateral, yaw);
    }

}

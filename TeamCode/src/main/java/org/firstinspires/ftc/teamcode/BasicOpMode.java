package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Driver Op Mode", group = "Driver Op Mode")
public class BasicOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Robot robot;

    private int armMotorPosition = -1, wristMotorPosition = -1;
    public static final double INTAKE_POWER = 1;
    public static final double WRIST_ANGLE_PER_SECOND = 40;
    public static final double EXTENSION_POWER = 1;

    Gamepad driverController;
    Gamepad armController;

    @Override
    public void runOpMode() {
        driverController = gamepad1;
        armController = gamepad2;
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

        if (armMotorPosition == -1) {
            armMotorPosition = robot.armMotor.getCurrentPosition();
        }

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
        telemetry.addData("Wrist current: ", robot.wristMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.update();

        double wristMovement = (0.01 * armController.left_stick_y);

        //if (wristMotorPosition == -1) {
        //    wristMotorPosition = robot.wristMotor.getCurrentPosition();
        //}

        telemetry.addData("wrist position", robot.wristMotor.getCurrentPosition());
        telemetry.addData("wrist target", robot.wristMotor.getTargetPosition());
        telemetry.update();
        int goToHere = 0;

        if (armController.dpad_up) {
            robot.wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wristMotor.setTargetPosition(goToHere);
            goToHere = goToHere + 1;
            //robot.wristMotor.setPower(0.5);
        }
        else if (armController.dpad_down) {
            robot.wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wristMotor.setTargetPosition(goToHere);
            goToHere = goToHere - 1;
            //robot.wristMotor.setPower(-0.5);
        }
        else {
            robot.wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wristMotor.setTargetPosition(goToHere);
            //robot.wristMotor.setPower(0);
            /*if (robot.wristMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
            {
                robot.wristMotor.setPower(0.1);
                robot.wristMotor.setTargetPosition(robot.wristMotor.getCurrentPosition());
                robot.wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }*/
        }
        /*
        if (Math.abs(wristMovement) <= 0.001) {
            // if joystick is under threshold or not pushed
            if (robot.wristMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
            {
                robot.wristMotor.setPower(0.1);
                robot.wristMotor.setTargetPosition(robot.wristMotor.getCurrentPosition());
                robot.wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        } else {
            // when pressed
            //wristMovement /= 20d; // UNCOMMENT THIS
            //wristMovement = 0;
            robot.wristMotor.setPower(wristMovement);
            robot.wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // wristMotorPosition = robot.wristMotor.getCurrentPosition();
        }

         */
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
        double axial = (1.0 * gamepad1.right_stick_x); // Forward Back
        double lateral = (0.6 * strafingSpeed); // Strafing (left right)
        double yaw = (-0.6 * gamepad1.left_stick_y); // Rotate
        robot.updateDriveMotors(axial, lateral, yaw);
    }

}

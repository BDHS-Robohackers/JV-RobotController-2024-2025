package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Driver Op Mode", group = "Driver Op Mode")
public class BasicOpMode extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx armMotor = null;
    private DcMotorEx armExtentionMotor = null;
    private ServoEx wristMotor = null;
    private DcMotor intakeMotor = null;

    private int armMotorPosition = 0;
    public static final double INTAKE_POWER = .5;
    public static final double WRIST_ANGLE_PER_SECOND = 40;
    public static final double EXTENSION_POWER = 1;

    @Override
    public void runOpMode() {

        final double MIN_ANGLE = -200;
        final double MAX_ANGLE = -175;

        // FOR COMPETITIONS: make armController gamepad2 so multiple people can use the robot
        Gamepad driverController = gamepad1;
        Gamepad armController = gamepad2;

        // Initialize the hardware variables. Note that the strings used here must
        // correspond
        // to the names assigned during the robot configuration step on the DS or RC
        // devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        armMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "arm_cool");
        armExtentionMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "arm_extension");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        wristMotor = new SimpleServo(hardwareMap, "wristy", MIN_ANGLE, MAX_ANGLE);

        // ########################################################################################
        // !!! IMPORTANT Drive Information. Test your motor directions. !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels
        // turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive,
        // it's important to ensure
        // that your motors are turning in the correct direction. So, start out with the
        // reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe
        // the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs
        // backward
        // Keep testing until ALL the wheels move the robot forward when you push the
        // left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armExtentionMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        /*leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

        armMotor.setTargetPositionTolerance(10);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtentionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Wait for the robot to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double deltaTime = elapsedTime.time();
            elapsedTime.reset();

            double max;

            double strafingspeed = gamepad1.left_trigger - gamepad1.right_trigger;
            double axial = (1.0 *gamepad1.left_stick_y); // Note: pushing stick forward gives negative value
            double lateral = (0.6*strafingspeed);
            double yaw = (0.6*gamepad1.right_stick_x);

            double armMovement = 0;
            armMovement = armController.right_stick_y; // TWO CONTROLLERS (controlled by #2)

            if (Math.abs(armMovement) <= 0.1) {
                // if joystick is under thresold or not pushed
                armMotor.setPower(1);
                armMotor.setTargetPosition(armMotorPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                // when pressed
                armMotor.setPower(armMovement);
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armMotorPosition = armMotor.getCurrentPosition();
            }

            if (armController.dpad_up) {
                wristMotor.rotateByAngle(WRIST_ANGLE_PER_SECOND * deltaTime);
            }
            if (armController.dpad_down) {
                wristMotor.rotateByAngle(-WRIST_ANGLE_PER_SECOND * deltaTime);
            }

            if (armController.x) {
                intakeMotor.setPower(INTAKE_POWER);
            } else if (armController.b) {
                intakeMotor.setPower(-INTAKE_POWER);
            } else {
                intakeMotor.setPower(0);
            }

            if (armController.dpad_right) {
                armExtentionMotor.setPower(EXTENSION_POWER);
            } else if (armController.dpad_left) {
                armExtentionMotor.setPower(-EXTENSION_POWER);
            } else {
                armExtentionMotor.setPower(0);
            }

            // Combine the joystick requests for each axis-motion to determine each wheel's
            // power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            // 1) First get all the motors to take to correct positions on the robot
            // by adjusting your Robot Configuration if necessary.
            // 2) Then make sure they run in the correct direction by modifying the
            // the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
                 leftFrontPower = driverController.x ? 1.0 : 0.0; // X gamepad
                 leftBackPower = driverController.a ? 1.0 : 0.0; // A gamepad
                 rightFrontPower = driverController.y ? 1.0 : 0.0; // Y gamepad
                 rightBackPower = driverController.b ? 1.0 : 0.0; // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }
}

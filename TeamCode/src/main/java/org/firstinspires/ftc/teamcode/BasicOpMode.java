package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
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
    private ServoEx wristMotor = null;
    private ServoEx handMotor = null;

    private int armMotorPosition = 0;

    @Override
    public void runOpMode() {

        final double MIN_ANGLE = 0;
        final double MAX_ANGLE = 270;
        boolean wasArmUnderThreshold = false;

        // Initialize the hardware variables. Note that the strings used here must
        // correspond
        // to the names assigned during the robot configuration step on the DS or RC
        // devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        armMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "arm_cool");
        wristMotor = new SimpleServo(hardwareMap, "wristy", MIN_ANGLE, MAX_ANGLE);
        handMotor = new SimpleServo(hardwareMap, "army", MIN_ANGLE, MAX_ANGLE);





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
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor.setTargetPositionTolerance(10);



        // Wait for the robot to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        final double armMotorCorrectionPower = 0.15;
        double armMotorPowerIncrementAdjustment = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to
            // rotate.
            double axial = -gamepad1.left_stick_y; // Note: pushing stick forward gives negative value
            double lateral = gamepad1.right_stick_x;
            double yaw = gamepad1.left_stick_x;

            boolean aPressedDown = gamepad1.a;

            telemetry.addData("Arm Motor Pos: ", armMotor.getCurrentPosition());
            telemetry.addData("SET Arm Motor Pos: ", armMotorPosition);
            if (gamepad1.y) {
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armMotor.setPower(1);
                armMotorPosition = armMotor.getCurrentPosition();
            }
            else if (gamepad1.a) {
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armMotor.setPower(-1);
                armMotorPosition = armMotor.getCurrentPosition();
            }
            else {
                if (armMotor.getPower() != 0)
                    armMotor.setPower(0);
                armMotor.setTargetPosition(armMotor.getCurrentPosition());
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                /*
                if (armMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
                {
                    armMotor.setTargetPosition(armMotorPosition);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                 */
                /*
                if (armMotor.getCurrentPosition() < armMotorPosition) {
                    armMotor.setPower(armMotorCorrectionPower - armMotorPowerIncrementAdjustment);
                    wasArmUnderThreshold = true;
                } else if (armMotor.getCurrentPosition() > armMotorPosition) {
                    if (wasArmUnderThreshold) {
                       armMotorPowerIncrementAdjustment = (armMotor.getCurrentPosition() - armMotorPosition) / (armMotorCorrectionPower - armMotorPowerIncrementAdjustment);
                    }
                    armMotor.setPower(-armMotorCorrectionPower + armMotorPowerIncrementAdjustment);
                }

                 */
            }

            final double STEP_SIZE = 0.01f;
            if (gamepad1.x) {
                handMotor.rotateByAngle(2);
            }

            if (gamepad1.b) {
                handMotor.rotateByAngle(-2);
            }

            if (gamepad1.dpad_up) {
                wristMotor.rotateByAngle(2);
            }

            if (gamepad1.dpad_down) {
                wristMotor.rotateByAngle(-2);
            }

            telemetry.addData("Hand Angle", handMotor.getAngle());

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
             leftFrontPower = gamepad1.x ? 1.0 : 0.0; // X gamepad
             leftBackPower = gamepad1.a ? 1.0 : 0.0; // A gamepad
             rightFrontPower = gamepad1.y ? 1.0 : 0.0; // Y gamepad
             rightBackPower = gamepad1.b ? 1.0 : 0.0; // B gamepad


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

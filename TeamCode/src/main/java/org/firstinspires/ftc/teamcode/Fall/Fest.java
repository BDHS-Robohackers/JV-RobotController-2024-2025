package org.firstinspires.ftc.teamcode.Fall;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "FallFest Driver Op Mode", group = "Driver Op Mode")
public class Fest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armMotor = null;
    private Servo pincher = null;

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor .class, "fl_drv");
        leftBackDrive = hardwareMap.get(DcMotor.class, "bl_drv");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr_drv");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br_drv");
        armMotor = hardwareMap.get(DcMotor.class, "arm_cool");
        pincher = hardwareMap.get(Servo.class, "pincher");
        // decrease left value if pincher is pinching too much
        // increase right value if pincher is expanding too far
        pincher.scaleRange(-0.3, 1.4);

        // Waiting for the play button to be pressed
        waitForStart();
        while (opModeIsActive()) {
            // Running
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to
            // rotate.
            double axial = -gamepad1.left_stick_x; // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_y;
            double yaw = gamepad1.right_stick_x;

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
             * leftFrontPower = gamepad1.x ? 1.0 : 0.0; // X gamepad
             * leftBackPower = gamepad1.a ? 1.0 : 0.0; // A gamepad
             * rightFrontPower = gamepad1.y ? 1.0 : 0.0; // Y gamepad
             * rightBackPower = gamepad1.b ? 1.0 : 0.0; // B gamepad
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

            armMotor.setPower(0);
            if (gamepad1.y) {
                armMotor.setPower(.5);
            }
            if (gamepad1.a) {
                armMotor.setPower(-.5);
            }

            if (gamepad1.x) {
                pincher.setPosition(1);
            } else {
                pincher.setPosition(0);
            }
        }

        // Code Finished
        armMotor.setPower(0);
        pincher.setPosition(0);
    }

}

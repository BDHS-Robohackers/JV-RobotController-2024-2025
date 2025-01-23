package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Robot {

    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public DcMotorEx armMotor;
    public DcMotorEx armExtentionMotor;
    public DcMotorEx wristMotor;
    public DcMotor intakeMotor;
    public TouchSensor intakeSensor;

    public Robot() {
        
    }

    public void initialize(HardwareMap hardwareMap) {
        final double MIN_ANGLE = -200;
        final double MAX_ANGLE = -175;

        try {
            leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
            leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
            rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
            armMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "arm_cool");
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
            wristMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "wristy");
            armExtentionMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "arm_extension");
            intakeSensor = hardwareMap.get(TouchSensor.class, "intake_touch");
        } catch (Exception e) {
            e.printStackTrace();
        }

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

        armMotor.setTargetPositionTolerance(10);
        wristMotor.setTargetPositionTolerance(10);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtentionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * @param axial forward is positive, backward is negative
     * @param lateral left is positive, right is negative
     * @param yaw clockwise is positive, counter-clockwise is negative
     */
    public void updateDriveMotors(double axial, double lateral, double yaw) {
        double max;

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
        //telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        //telemetry.update();
    }

}

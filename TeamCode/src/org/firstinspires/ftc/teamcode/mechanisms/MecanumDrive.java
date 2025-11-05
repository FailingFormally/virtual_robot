package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Controls a Mecanum drive with four motors and wheels with 45 degree bearings.
 * This allows driving in any direction.
 * <p>
 * see Learn Java for FTC by Alan G. Smith, Section 20.2.
 */
public class MecanumDrive {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private Telemetry telemetry;

    /**
     * @param telemetry - Pass the telemetry instance from the OpMode to get telemetry output.
     */
//    public MecanumDrive(Telemetry telemetry) {
//        this.telemetry = telemetry;
//    }

    /**
     * Initialize motors to run using encoder. Configure your robot
     * for the names below, or adjust them.
     *
     * @param hardwareMap - The hardware configuration information
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        frontLeftMotor = hardwareMap.dcMotor.get("front_left_motor");
        frontRightMotor = hardwareMap.dcMotor.get("front_right_motor");
        backLeftMotor = hardwareMap.dcMotor.get("back_left_motor");
        backRightMotor = hardwareMap.dcMotor.get("back_right_motor");

        // Left side is mounted such that these must be reversed
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Sets the power for each motor.  Turns are determined by the relative speeds of the motor.
     *
     * @param frontLeftPower  - Value between 1 and -1 for motor speed
     * @param frontRightPower - Value between 1 and -1 for motor speed
     * @param backLeftPower   - Value between 1 and -1 for motor speed
     * @param backRightPower  - Value between 1 and -1 for motor speed
     * @param speedMultiplier - Adjust speed, e.g. half speed (0.5), full speed (1.0)
     */
    private void setPowers(double frontLeftPower, double frontRightPower,
                           double backLeftPower, double backRightPower,
                           double speedMultiplier) {

        // Ensure maxSpeed is a positive value greater than 1.
        speedMultiplier = Math.abs(speedMultiplier);
        if (speedMultiplier > 1) {
            speedMultiplier = 1.0;
        }

        // This normalizes the values so that the maximum speed is one.
        double maxSpeed = 1.0;
        maxSpeed = Math.max(maxSpeed, Math.abs(frontLeftPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(frontRightPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(backLeftPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(backRightPower));

        frontLeftPower /= maxSpeed;
        frontRightPower /= maxSpeed;
        backLeftPower /= maxSpeed;
        backRightPower /= maxSpeed;

        // Now apply the speed multiplier
        // This can be used to go in half speed (0.5), full speed (1.0) or anything between.
        frontLeftPower *= speedMultiplier;
        frontRightPower *= speedMultiplier;
        backLeftPower *= speedMultiplier;
        backRightPower *= speedMultiplier;

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);

        telemetry.addData("Front Left Motor Power", frontLeftPower);
        telemetry.addData("Back Left Motor Power", backLeftPower);
        telemetry.addData("Front Right Motor Power", frontRightPower);
        telemetry.addData("Back Right Motor Power", backRightPower);
    }

    /**
     * This sets the direction of movement, as well as the speed and direction of rotation.
     * It assumes a speedMultipler (max value) of 1.0.
     * Thanks to FTC16072 for sharing this code!!
     * <p>
     * {@snippet lang = "java":
     * double forward = -gamepad1.left_stick_y;
     * double right = gamepad1.left_stick_x;
     * double rotate = gamepad1.right_stick_x;
     *
     * drive.drive(forward, right, rotate);
     *}
     *
     * @param forward - A value between 1 and -1 for speed in the forward/reverse direction.
     * @param right   - A value between 1 and -1 for speed in the right/left direction.
     * @param rotate  - A value between 1 and -1 for rotation speed
     */
    public void drive(double forward, double right, double rotate) {
        drive(forward, right, rotate, 1.0);
    }

    /**
     * This sets the direction of movement, as well as the speed and direction of rotation.
     * It assumes a speedMultipler (max value) of 1.0.
     * Thanks to FTC16072 for sharing this code!!
     * <p>
     * {@snippet lang = "java":
     * double forward = -gamepad1.left_stick_y;
     * double right = gamepad1.left_stick_x;
     * double rotate = gamepad1.right_stick_x;
     * double maxSpeed = 1.0;
     *
     * drive.drive(forward, right, rotate, maxSpeed);
     *}
     *
     * @param forward         - A value between 1 and -1 for speed in the forward/reverse direction.
     * @param right           - A value between 1 and -1 for speed in the right/left direction.
     * @param rotate          - A value between 1 and -1 for rotation speed
     * @param speedMultiplier - A positive value less than 1.  A value of 0.5 would result in half speed.
     */
    public void drive(double forward, double right, double rotate, double speedMultiplier) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backLeftPower = forward - right + rotate;
        double backRightPower = forward + right - rotate;

        setPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower, speedMultiplier);
    }
}
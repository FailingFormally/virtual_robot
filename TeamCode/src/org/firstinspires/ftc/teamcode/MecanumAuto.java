package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 * Adapted from MecanumDemo and AutoDriveByGyro_Linear
 *
 */
@Autonomous(name = "Mecanum Auto Updated", group = "MecanumBot")
public class MecanumAuto extends LinearOpMode {

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 2.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.5;     // Max turn speed to limit turn rate.
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.1;     // Larger is more responsive, but also less stable.
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable.

    DcMotor backLeftMotor;
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DistanceSensor frontDistance;
    DistanceSensor leftDistance;
    DistanceSensor rightDistance;
    DistanceSensor backDistance;
    IMU imu;
    ColorSensor colorSensor;
    OctoQuad octoQuad;


    public void initialize() {
        // Motors
        backLeftMotor = hardwareMap.dcMotor.get("back_left_motor");
        frontLeftMotor = hardwareMap.dcMotor.get("front_left_motor");
        frontRightMotor = hardwareMap.dcMotor.get("front_right_motor");
        backRightMotor = hardwareMap.dcMotor.get("back_right_motor");

        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        // Sensors
        frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");

        imu = hardwareMap.get(IMU.class, "imu");

        colorSensor = hardwareMap.colorSensor.get("color_sensor");
        octoQuad = hardwareMap.get(OctoQuad.class, "octoquad");
    }

    /**
     * A utility method to set all motors to the same mode.
     * @param mode
     */
    private void setMotorModes(DcMotor.RunMode mode) {
        backLeftMotor.setMode(mode);
        frontLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
        backRightMotor.setMode(mode);
    }

    private boolean motorsAreBusy() {
        return backLeftMotor.isBusy() && frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backRightMotor.isBusy();
    }

    public void runOpMode(){
        initialize();
        
        telemetry.addData("Press Start When Ready","");
        telemetry.update();

        waitForStart();

        // Zero degrees will be the heading when start is pushed.
        imu.resetYaw();

        driveStraight(DRIVE_SPEED, 12, 0);
        turnToHeading(TURN_SPEED, 90);
        driveStraight(DRIVE_SPEED, 12, 90);
        turnToHeading(TURN_SPEED, 45);
        driveStraight(DRIVE_SPEED, -12, 45);
        stopMotors();
    }

    /**
     *  Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the OpMode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraight(double maxDriveSpeed, double distance, double heading) {
        if (opModeIsActive()) {

            // Determine new target position for each motor
            int moveCounts = (int)(distance * COUNTS_PER_INCH);

            // Set Target FIRST, then turn on RUN_TO_POSITION
            backLeftMotor.setTargetPosition( backLeftMotor.getCurrentPosition() + moveCounts );
            frontLeftMotor.setTargetPosition( frontLeftMotor.getCurrentPosition() + moveCounts );
            frontRightMotor.setTargetPosition( frontRightMotor.getCurrentPosition() + moveCounts );
            backRightMotor.setTargetPosition( backRightMotor.getCurrentPosition() + moveCounts );

            setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

            setMotorPowers(maxDriveSpeed, 0, 0);

            while (opModeIsActive() && motorsAreBusy()) {
                // Correct for Heading here
                double turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                if (distance < 0) {
                    turnSpeed *= -1.0;
                }

                setMotorPowers(maxDriveSpeed, 0 , turnSpeed);
            }

        }

       stopMotors();
       setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /**
     *  Spin on the central axis to point in a new direction.
     *  <p>
     *  Move will stop if either of these conditions occur:
     *  <p>
     *  1) Move gets to the heading (angle)
     *  <p>
     *  2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        double headingError = getHeadingError(heading);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            double turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            setMotorPowers(0, 0, turnSpeed);

            // Refresh headingError
            headingError = getHeadingError(heading);
        }

        // Stop all motion;
        stopMotors();
    }

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        double headingError = getHeadingError(desiredHeading);

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public double getHeadingError(double desiredHeading) {
        // Determine the heading current error
        double headingError = desiredHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;
        return headingError;
    }


    protected void setMotorPowers(double px, double py, double pa) {
        if (Math.abs(pa) < 0.05) pa = 0;
        double p1 = -px + py - pa;
        double p2 = px + py + -pa;
        double p3 = -px + py + pa;
        double p4 = px + py + pa;
        double max = Math.max(1.0, Math.abs(p1));
        max = Math.max(max, Math.abs(p2));
        max = Math.max(max, Math.abs(p3));
        max = Math.max(max, Math.abs(p4));
        p1 /= max;
        p2 /= max;
        p3 /= max;
        p4 /= max;
        backLeftMotor.setPower(p1);
        frontLeftMotor.setPower(p2);
        frontRightMotor.setPower(p3);
        backRightMotor.setPower(p4);
        telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
        telemetry.addData("Heading", " %.1f", getHeading());
        telemetry.addData("Heading Error", " %.1f", getHeadingError(90));
        telemetry.addData("Angular Velocity", "%.1f", getAngularVelocity());
        telemetry.addData("Front Distance", " %.1f", frontDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("Left Distance", " %.1f", leftDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("Right Distance", " %.1f", rightDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("Back Distance", " %.1f", backDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("Encoders"," %d %d %d %d", backLeftMotor.getCurrentPosition(), frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        telemetry.addData("Octoquad", "%d %d %d %d", octoQuad.readSinglePosition(0),
                octoQuad.readSinglePosition(1), octoQuad.readSinglePosition(2),
                octoQuad.readSinglePosition(3));
        telemetry.update();
    }

    protected void stopMotors() {
        backLeftMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public float getAngularVelocity() {
        return imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
    }
}

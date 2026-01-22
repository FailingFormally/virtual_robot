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
@Autonomous(name = "Mecanum Auto Updated Simplified", group = "MecanumBot")
public class MecanumAuto extends MecanumAutoLinearOpMode {

    @Override
    public void runRoutine() {
        driveStraight(DRIVE_SPEED, 12, 0);
        turnToHeading(TURN_SPEED, 90);
        driveStraight(DRIVE_SPEED, 12, 90);
        turnToHeading(TURN_SPEED, 45);
        driveStraight(DRIVE_SPEED, -12, 45);
    }
}

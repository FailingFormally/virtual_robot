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
@Autonomous(name = "Improved Auto Test", group = "MecanumBot")
public class MecanumAuto extends MecanumAutoLinearOpMode {

    enum AutoRoutine {
        BlueLong, BlueShort, RedLong, RedShort
    }

    private AutoRoutine autoSelected = AutoRoutine.RedShort;

    @Override
    public void onInitLoop() {
        telemetry.addData("Selected Auto", autoSelected);
        telemetry.addData("Press D-Pad Up", "BlueLong");
        telemetry.addData("Press D-Pad Down", "BlueShort");
        telemetry.addData("Press D-Pad Left", "RedLong");
        telemetry.addData("Press D-Pad Right", "RedShort");
        telemetry.update();

        // Check for controller input to change selection
        if (gamepad1.dpad_up) {
            autoSelected = AutoRoutine.BlueLong;
        } else if (gamepad1.dpad_down) {
            autoSelected = AutoRoutine.BlueShort;
        } else if (gamepad1.dpad_left) {
            autoSelected = AutoRoutine.RedLong;
        } else if (gamepad1.dpad_right) {
            autoSelected = AutoRoutine.RedShort;
        }
    }

    @Override
    public void runRoutine() {

        switch(autoSelected) {
            case RedShort -> runRedShortAuto();
            case RedLong -> runRedLongAuto();
            case BlueShort -> runBlueShortAuto();
            case BlueLong -> runBlueLongAuto();
        }
    }

    private void runRedShortAuto() {
        driveStraight(DRIVE_SPEED, 48, 0);
        // Add launch here
        turnToHeading(TURN_SPEED, 45);
        driveStraight(DRIVE_SPEED, 10, 45);
        turnToHeading(TURN_SPEED, 135);
        driveStraight(0.3, 40, 135);
        driveStraight(DRIVE_SPEED, -40, 135);
        turnToHeading(TURN_SPEED, 45);
        driveStraight(DRIVE_SPEED, -10, 45);
        // turn toward goal again
        turnToHeading(TURN_SPEED, 0);
        // add launch here
    }

    private void runRedLongAuto() {
        telemetry.addData("I don't know the Auto routine for:", autoSelected);
        telemetry.update();
        sleep(2000);
    }

    /**
     * Same as `runRedShortAuto` but the angles are reversed.
     */
    private void runBlueShortAuto() {
        driveStraight(DRIVE_SPEED, 48, 0);
        // Add launch here
        turnToHeading(TURN_SPEED, -45);
        driveStraight(DRIVE_SPEED, 10, -45);
        turnToHeading(TURN_SPEED, -135);
        driveStraight(0.3, 40, -135);
        driveStraight(DRIVE_SPEED, -40, -135);
        turnToHeading(TURN_SPEED, -45);
        driveStraight(DRIVE_SPEED, -10, -45);
        // turn toward goal again
        turnToHeading(TURN_SPEED, 0);
        // add launch here
    }

    private void runBlueLongAuto() {
        telemetry.addData("I don't know the Auto routine for:", autoSelected);
        telemetry.update();
        sleep(2000);
    }
}

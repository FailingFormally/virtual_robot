package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@TeleOp(name="Drive with MecanumDriver class")
public class DriveOpMode extends OpMode {

    MecanumDrive drive = new MecanumDrive();

    @Override
    public void init() {
        drive.init(hardwareMap, this.telemetry);
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        drive.drive(forward, right, rotate);
        telemetry.update();

    }
}

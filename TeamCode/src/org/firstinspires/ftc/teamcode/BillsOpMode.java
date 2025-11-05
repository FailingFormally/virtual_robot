package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Turbo;

@TeleOp(group="Bill", name="Gamepad Driving")
public class BillsOpMode extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    // Create a Turbo system and default state to true
    Turbo turbo = new Turbo(gamepad1, true);

    @Override
    public void init() {
        drive.init(hardwareMap, this.telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper) { turbo.disable(); }
        if (gamepad1.right_bumper) { turbo.enable(); }

        // Output turbo status
        telemetry.addData("Turbo", turbo.getEnabled());

        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        drive.drive(forward, right, rotate, turbo.getSpeed());
        telemetry.update();
    }

}

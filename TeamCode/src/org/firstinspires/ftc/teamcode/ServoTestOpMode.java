package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Test", group="Test")
public class ServoTestOpMode extends OpMode {

    private Servo servo;

    private Servo.Direction direction = Servo.Direction.FORWARD;

    private double targetPostion = 0;

    private boolean wasUp, wasDown;

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");
        targetPostion = servo.getPosition();
    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {

        if (gamepad1.dpad_up && !wasUp && targetPostion < 1) {
            targetPostion += 0.01;
        }
        wasUp = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !wasDown && targetPostion > 0) {
            targetPostion -= 0.01;
        }
        wasDown = gamepad1.dpad_down;

        if (gamepad1.left_bumper) {
            if (direction == Servo.Direction.FORWARD) {
                direction = Servo.Direction.REVERSE;
            } else {
                direction = Servo.Direction.FORWARD;
            }
        }

        if (gamepad1.x) {
            servo.setDirection(direction);
            servo.setPosition(targetPostion);
        }

        telemetry.addLine("Press X to go to target position");
        telemetry.addLine("Press Left Bumper to change direction");
        telemetry.addData("Current Position", servo.getPosition());
        telemetry.addData("Target Position", targetPostion);
        telemetry.addData("Current Direction", direction);
        telemetry.update();

    }
}

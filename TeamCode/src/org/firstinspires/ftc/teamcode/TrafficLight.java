package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto Traffic Light")
public class TrafficLight extends OpMode {

    ElapsedTime time = new ElapsedTime();

    enum LightState {
        RED, YELLOW, GREEN
    }

    private LightState lightState = LightState.GREEN;

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {

    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
        switch (lightState) {
            case GREEN:
                gamepad1.setLedColor(0.0, 1.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS);
                if (time.seconds() > 3.0) {
                    lightState = LightState.YELLOW;
                }
                break;
            case YELLOW:
                gamepad1.setLedColor(1.0, 1.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS);
                if (time.seconds() > 5) {
                    lightState = LightState.RED;
                }
                break;
            case RED:
                gamepad1.setLedColor(1.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS);
                if (time.seconds() > 10) {
                    lightState = LightState.GREEN;
                    time.reset();
                }
                break;
        }
    }
}

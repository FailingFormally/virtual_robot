package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Turbo {

    public Turbo(Gamepad gamepad, boolean defaultState) {
        this.gamepad = gamepad;
        this.turboEnabled = defaultState;
    }
    private boolean turboEnabled = false;

    private final Gamepad gamepad;

    public void enable() {
        turboEnabled = true;
        gamepad.rumbleBlips(3);
    }

    public void disable() {
        turboEnabled = false;
        gamepad.rumbleBlips(1);
    }

    public boolean getEnabled() {
        return turboEnabled;
    }

    public double getSpeed() {
        if (getEnabled()) {
            return 1.0;
        } else {
            return 0.5;
        }
    }
}

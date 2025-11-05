package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class PIDMotorControlOpMode extends OpMode {

    final double tolerance = 0.1;

    DcMotorEx launcher = null;

    /**
     * Tracking the state of DPAD UP and DOWN
     */
    boolean wasUp, wasDown;

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    private double targetVelocity = 1000.0;

    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */

    /**
     * The states of our Launching Mechanism.
     */
    private enum LaunchState {
        /**
         * The default state
         */
        IDLE,
        /**
         * Once the launch is triggered, we stay in the SPIN_UP state until the
         * motors are running at the target velocity.
         */
        SPIN_UP,
        /**
         * Once motors hit target velocity, we hit this state so that we can trigger the launch.
         * Ideally we would trigger servos to put the artifact in place here.
         */
        LAUNCH,
        /**
         * In this state, we are actively waiting for the launch to occur.
         * From here, we probably need to return to SPIN_UP to get back to launch
         * velocity.
         */
        LAUNCHING
    }

    private LaunchState launchState = LaunchState.IDLE;

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {
        launcher = hardwareMap.get(DcMotorEx.class, "motor");
        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
        // Increase the Target velocity by 100 rpm
        if (gamepad1.dpad_up && !wasUp) {
            targetVelocity += 100;  // targetVelocity = targetVelocity + 100;
        }
        wasUp = gamepad1.dpad_up;

        // Decrease the target velocity by 100 rpm
        if (gamepad1.dpad_down && !wasDown) {
            targetVelocity -= 100;
        }
        wasDown = gamepad1.dpad_down;

        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Actual Velocity", launcher.getVelocity());
        telemetry.addData("Launch State", launchState.toString());

        launch(gamepad1.yWasPressed());

        telemetry.update();


    }

    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(targetVelocity);
                if (launcher.getVelocity() > targetVelocity - tolerance) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                launcher.setVelocity(targetVelocity);
                // For now, if velocity drops, we probably fired... go back to SPIN_UP
                if (launcher.getVelocity() < (targetVelocity + tolerance)) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
        }
    }
}

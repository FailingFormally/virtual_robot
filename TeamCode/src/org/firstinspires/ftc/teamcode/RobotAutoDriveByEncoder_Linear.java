/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.mechanisms.YeeterKing;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Auto Select", group="Auto")
public class RobotAutoDriveByEncoder_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor front_left_Motor    = null;
    private DcMotor front_right_Motor  = null;
    private DcMotor back_left_Motor    = null;
    private DcMotor back_right_Motor   = null;

    private ElapsedTime     runtime = new ElapsedTime();

    private String autoSelected = "RedLong"; // Default autonomous mode

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.09449 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

//    private YeeterKing yeeter = new YeeterKing();

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        front_left_Motor = hardwareMap.dcMotor.get("front_left_motor");
        front_right_Motor = hardwareMap.dcMotor.get("front_right_motor");
        back_left_Motor = hardwareMap.dcMotor.get("back_left_motor");
        back_right_Motor = hardwareMap.dcMotor.get("back_right_motor");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        front_left_Motor.setDirection(DcMotor.Direction.REVERSE);
        front_right_Motor.setDirection(DcMotor.Direction.FORWARD);
        back_left_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right_Motor.setDirection(DcMotorSimple.Direction.FORWARD);

        front_left_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_right_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                front_left_Motor.getCurrentPosition(),
                front_right_Motor.getCurrentPosition(),
                back_left_Motor.getCurrentPosition(),
                back_right_Motor.getCurrentPosition());
        telemetry.update();

        // This loop runs during the INIT phase
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Selected Auto", autoSelected);
            telemetry.addData("Press D-Pad Up", "BlueLong");
            telemetry.addData("Press D-Pad Down", "BlueShort");
            telemetry.addData("Press D-Pad Left", "RedLong");
            telemetry.addData("Press D-Pad Right", "RedShort");
            telemetry.update();

            // Check for controller input to change selection
            if (gamepad1.dpad_up) {
                autoSelected = "BlueLong";
            } else if (gamepad1.dpad_down) {
                autoSelected = "BlueShort";
            } else if (gamepad1.dpad_left) {
                autoSelected = "RedLong";
            } else if (gamepad1.dpad_right) {
                autoSelected = "RedShort";
            }
            sleep(50); // Add a small delay to avoid excessive polling
        }
        // Wait for the game to start (driver presses START)
        waitForStart();

        if (opModeIsActive()) {
            switch (autoSelected) {
                case "RedShort":
                    runRedShortAuto();
                    break;
                case "BlueLong":
                    runBlueLongAuto();
                    break;
                case "RedLong":
                    runRedLongAuto();
                    break;
                case "BlueShort":
                    runBlueShortAuto();
                    break;
                default:
// Default to a safe routine, or do nothing
                    telemetry.addData("Error", "Invalid auto selected.");
                    telemetry.update();
                    break;
            }
        }


//        yeeter.launch(true,800);
        sleep(1000);  // pause to display final telemetry message.
    }
    private void runRedLongAuto()
    {
        telemetry.addData("Running", "Red Long Auto");
        telemetry.update();
        encoderDrive(DRIVE_SPEED,  183,  183, 5.0);
        encoderDrive(TURN_SPEED,   20,-20 , 4.0);
    }
    private void runBlueLongAuto()
    {
        telemetry.addData("Running", "Blue Long Auto");
        telemetry.update();
        encoderDrive(DRIVE_SPEED,  183,  183, 5.0);
        encoderDrive(TURN_SPEED,   -20,20 , 4.0);
    }

    private void runBlueShortAuto() {
        telemetry.addData("Running", "Blue Short Auto");
        telemetry.update();
        encoderDrive(DRIVE_SPEED, -48, -48, 5.0);
    }
    private void runRedShortAuto()
    {
        telemetry.addData("Running", "Red Short Auto");
        telemetry.update();
        encoderDrive(DRIVE_SPEED, -48, -48, 5.0);
    }


    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = front_left_Motor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = front_right_Motor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            front_left_Motor.setTargetPosition(newLeftTarget);
            back_right_Motor.setTargetPosition(newRightTarget);
            front_right_Motor.setTargetPosition(newRightTarget);
            back_left_Motor.setTargetPosition(newLeftTarget);

            // Turn On RUN_TO_POSITION
            front_left_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_left_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            front_left_Motor.setPower(Math.abs(speed));
            front_right_Motor.setPower(Math.abs(speed));
            back_right_Motor.setPower(Math.abs(speed));
            back_left_Motor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (front_left_Motor.isBusy() && front_right_Motor.isBusy() && back_left_Motor.isBusy() && back_right_Motor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d %7d %7d" ,
                        front_left_Motor.getCurrentPosition(),
                        front_right_Motor.getCurrentPosition(),
                        back_left_Motor.getCurrentPosition(),
                        back_left_Motor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            front_left_Motor.setPower(0);
            front_right_Motor.setPower(0);
            back_left_Motor.setPower(0);
            back_right_Motor.setPower(0);

            // Turn off RUN_TO_POSITION
            front_left_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_left_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}

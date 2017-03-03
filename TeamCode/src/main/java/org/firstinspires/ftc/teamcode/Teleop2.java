/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * This file provides basic Telop driving for a Ghostbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Ghostbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwareGhostbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a Ghostbot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 * THIS IS CREATED BY TEAM MEMBER ANVAY BUCH - COMPILE AND UPLOAD THIS CODE TO YOUR PHONE
 */


/**This code has been slightly modied by Leo Kiefer on 12/3/16 to better control the 2016-17 ftc robot.
 */

/*
Hi! Its Leo againn editing for the new 16-17 robot on 1/27/17!
A lot of the old stuff changed, so almost all of he comments below make no sense any more!

Josh edited this code on February 17 to add in the new shooter controls that Will wanted but
apparently didnt' want to lend the robot to me for. Sorry if I break anything.

(Josh 24 Feb 2017)

The buttons are:

Gamepad 1:
[Right Stick] Right Drive Motor
[Left Stick] Left Drive Motor
[Right Bumper] Gear Go Faster
[Left Bumper] Gear Go Slower
[R + L Bumper] Gear Reset
[Y] Raise Lift
[A] Lower Lift

Gamepad 2:
[X] Start/Stop Spinner of Death
[Y] Reset Shooter
[A] Lower Shooter into Loading Position
[B] Fire Shooter
 */

@TeleOp(name="Ghostbot: Teleop 2", group="Ghostbot")
public class Teleop2 extends OpMode{

    /* Declare OpMode members. */
    HardwareGhostbot robot       = new HardwareGhostbot(); // use the class created to define a Ghostbot's hardware
                                                         // could also use HardwareGhostbotMatrix class.
    final double START_SPEED = 1.0;                     //Starting Value of the drive speed
    final double SPEED_INCREMENT = 0.1;                 //how much the speed changes with the D-Pad
    double driveSpeed = START_SPEED;                    //speed it can drive at
    boolean speedvar = false;
    boolean hairOn = false;
    boolean shooting = false;
    boolean lowered = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //Just displays messages on the phone for debugging purposes
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    public void motorTime(DcMotor motor, double power, int ms) {
        motor.setPower(power);
        WaitMs(ms);
        motor.setPower(0);
    }

    @Override
    public void loop() {
        double left;
        double right;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -gamepad1.left_stick_y;   //only the first gamepad can drive
        right = - gamepad1.right_stick_y;
        robot.leftMotor.setPower(left*driveSpeed);
        robot.rightMotor.setPower(right*driveSpeed);

        //reset the shooter if g2.y is pressed
        if(gamepad2.b && !shooting){
            shooting = true;
            if(lowered)
                lowered = false;
            telemetry.addData("Launcher Status: ", "Launching!");
            motorTime(robot.launcher, 1, 600);
            shooting = false;
        }
        if(gamepad2.a && !lowered){
            lowered = true;
            robot.launcher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.launcher.setTargetPosition(240);
            robot.launcher.setPower(0.7);
            while(robot.launcher.isBusy())
                telemetry.addData("Launcher Status: ", "Lowering!");
        }
        if(gamepad2.a && lowered){
            robot.launcher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.launcher.setTargetPosition(0);
            robot.launcher.setPower(0.7);
            while(robot.launcher.isBusy())
                telemetry.addData("Launcher Status: ", "Raising!");
            robot.launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lowered = false;
        }

        //(make sure the driveSpeed stays between 0 and 100)
        //use D-Pad up/down to change the drive speed
        if (!speedvar) {
            if ((gamepad1.right_bumper) && driveSpeed < 1.3)
                driveSpeed += SPEED_INCREMENT;
            speedvar = true;
            if ((gamepad1.left_bumper) && driveSpeed > 0)
                driveSpeed -= SPEED_INCREMENT;
            speedvar = true;
        }
        if (!gamepad1.left_bumper && !gamepad1.right_bumper)
            speedvar = false;

        //resets the drive speed if the right d-pad button is pressed
        if(gamepad1.right_bumper && gamepad1.left_bumper)
            driveSpeed=START_SPEED;

        // Use gamepad buttons to move the arm up (Y) and down (A)
        if (gamepad1.y) {
            robot.lift.setPower(1);

        }else if (gamepad1.a) {
            robot.lift.setPower(-1);

        }else{
            robot.lift.setPower(0);
        }

        //Use gamepad button [X] to activate hand-murderer
        if (!hairOn && gamepad2.x) {
            robot.spinner.setPower(1);
            hairOn = true;
        }
        if (hairOn && gamepad2.x)
            robot.spinner.setPower(0);
        hairOn = false;

        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.addData("Drive Speed: ",driveSpeed);
        telemetry.addData("Stick: ", -gamepad2.right_stick_x);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void WaitMs(int milliseconds){
        try {
            Thread.sleep(milliseconds);
        } catch (Exception e) {
        }
    }

}

//dank memes bruh
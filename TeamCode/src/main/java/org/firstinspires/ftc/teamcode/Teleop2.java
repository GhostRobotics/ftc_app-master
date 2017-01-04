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
 *
 * Both gamepads now trigger the same actions with the same buttons. The only exception is that
 * the 2 driving motors can only be controlled from gamepad 1 and only gamepad 2 can control the beacon
 * pushers.
 *
 * To reset the claw servo arms to their starting positions, pess the B button on BOTH CONTROLLERS!
 *
 * Button Guide:
 * FUNCTION           BUTTON          CONTROLLERS (1 or 2)
 * =======================================================
 * move left           left
 * wheel back          stick            ONLY #1
 * and forth          (up/down)
 * (same stuff for the right wheel, but with other gamepad1 stick)
 *
 * Increase/Decrease  dpad up/down      Either
 * Driving speed       (hold down)
 *
 * Reset driving
 * speed to its        dpad-right       Either
 *original value
 *
 * Beacon           left/right
 * Pushers             triggers        ONLY #2
 *
 * Extend Claws         Y               Either
 *
 * Lower claws          A               Either
 *
 * Close Claws       right bumper       Either
 *
 * Open Claws        Left Bumper        Either
 *
 * Reset claws          B               BOTH controllers (at the same time)
 */

@TeleOp(name="Ghostbot: Teleop 2", group="Ghostbot")
public class Teleop2 extends OpMode{

    /* Declare OpMode members. */
    HardwareGhostbot robot       = new HardwareGhostbot(); // use the class created to define a Ghostbot's hardware
                                                         // could also use HardwareGhostbotMatrix class.
    final double CLAW_RESET = -0.5;                     //claw servo reset and starting position
    double          clawOffset  = CLAW_RESET;           // Servo mid position
    final double    CLAW_SPEED  = 0.01; // sets rate to move servo

    final double START_SPEED = 1.0;                     //Starting Value of the drive speed
    final double SPEED_INCRIMENT = 0.1;                 //how much the speed changes with the D-Pad
    double driveSpeed = START_SPEED;                    //speed it can drive at
    int speedvar = 0;

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
    @Override
    public void loop() {
        double left;
        double right;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = gamepad1.left_stick_y;   //only the first gamepad can drive
        right = -gamepad1.right_stick_y;
        robot.leftMotor.setPower(left*driveSpeed);
        robot.rightMotor.setPower(right*driveSpeed);

        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad1.right_bumper || gamepad2.right_bumper)
            clawOffset += CLAW_SPEED;
        else if (gamepad1.left_bumper || gamepad2.left_bumper)
            clawOffset -= CLAW_SPEED;

        //reset the claw servos if the B button is pressed ON BOTH CONTROLLERS
        if(gamepad1.b && gamepad2.b)
            clawOffset = CLAW_RESET;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        robot.claw1.setPosition(robot.MID_SERVO + clawOffset);
        robot.claw2.setPosition(robot.MID_SERVO - clawOffset);

        //(make sure the driveSpeed stays between 0 and 100)
        //use D-Pad up/down to change the drive speed
        if (speedvar == 0) {
            if ((gamepad1.dpad_up || gamepad2.dpad_up) && driveSpeed < 1.0)
                driveSpeed += SPEED_INCRIMENT;
            speedvar = 1;
            if ((gamepad1.dpad_down || gamepad2.dpad_down) && driveSpeed > 0)
                driveSpeed -= SPEED_INCRIMENT;
            speedvar = 1;
        }
        if (!gamepad1.dpad_down && !gamepad2.dpad_down && !gamepad1.dpad_up && !gamepad2.dpad_up)
            speedvar = 0;
        //resets the drive speed if the right d-pad button is pressed
        if(gamepad1.dpad_right || gamepad2.dpad_right)
            driveSpeed=START_SPEED;


        // Use gamepad buttons to move the arm up (Y) and down (A)
        if (gamepad1.y || gamepad2.y) {
            robot.arm1.setPower(-robot.ARM_UP_POWER);
            robot.arm2.setPower(-robot.ARM_UP_POWER);
            robot.arm3.setPower(robot.ARM_UP_POWER);
            robot.arm4.setPower(robot.ARM_UP_POWER);

        }else if (gamepad1.a || gamepad2.a) {
            robot.arm1.setPower(-robot.ARM_DOWN_POWER);
            robot.arm2.setPower(-robot.ARM_DOWN_POWER);
            robot.arm3.setPower(robot.ARM_DOWN_POWER);
            robot.arm4.setPower(robot.ARM_DOWN_POWER);

        }else {
            robot.arm1.setPower(0.0);
            robot.arm2.setPower(0.0);
            robot.arm3.setPower(0.0);
            robot.arm4.setPower(0.0);
        }

        //use the gamepad2 triggers to control the positions of the beacon pushers
        robot.bp1.setPosition( robot.MID_SERVO - (gamepad2.right_trigger - 0.5f));
        robot.bp2.setPosition(robot.MID_SERVO + (gamepad2.left_trigger - 0.5f));

        // Send telemetry message to signify robot running;
        telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.addData("Drive Speed: ",driveSpeed);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

//dank memes bruh
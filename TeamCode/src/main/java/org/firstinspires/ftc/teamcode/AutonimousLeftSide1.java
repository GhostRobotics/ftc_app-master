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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Autonomous: LeftSide 1", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
public class AutonimousLeftSide1 extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private final double CIRCUMFERENCE=4.0*Math.PI;
    private final double TURNING_CONSTANT=21.0;

    // private DcMotor leftMotor = null;
    // private DcMotor rightMotor = null;
    HardwareGhostbot robot = new HardwareGhostbot(); //Defines robot as HardwareGhostbot

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        int TICKS_PER_REV = 1440;


        //set motors to RUN_TO_POSITION
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

    public void DriveForward(double power) {
        robot.leftMotor.setPower(power);
        robot.rightMotor.setPower(power);
    }

    public void StopDriving() {
        DriveForward(0);
    }

    public void TurnLeft(double power) {
        robot.leftMotor.setPower(-power);
        robot.rightMotor.setPower(power);
    }

    public void TurnRight(double power) {
        TurnLeft(-power);
    }

    public void DriveForwardTicks(double power, int distance) {
        //Set target position
        robot.leftMotor.setTargetPosition(distance);
        robot.rightMotor.setTargetPosition(distance);

        //set drive power
        DriveForward(power);

        //Confirm that motors are done
        while(robot.leftMotor.isBusy() && robot.rightMotor.isBusy()) {
        }

        //stop
        //StopDriving();

        //reset for next go-around
        ResetEncoders();
    }

    public void DriveBackwardTicks(double power, int distance){
        DriveForwardTicks(power,-distance);
    }

    public void DriveForwardTurns(double power, double turns){
        DriveForwardTicks(power,(int)(turns*1440));
    }
    public void DriveBackwardTurns(double power, double turns){
        DriveForwardTurns(power,-turns);
    }
    public void DriveForwardDistance(double power, double distance){
        double turns=distance/CIRCUMFERENCE;

        DriveForwardTurns(power,turns);
    }
    public void DriveBackwardDistance(double power, double distance){
        DriveForwardDistance(power, -distance);
    }

    public void TurnInPlaceRight(double power, int degrees){
        degrees*=TURNING_CONSTANT;

        //Set target position
        robot.leftMotor.setTargetPosition(degrees);
        robot.rightMotor.setTargetPosition(-degrees);

        //set drive power
        DriveForward(power);

        //Confirm that motors are done
        while(robot.leftMotor.isBusy() && robot.rightMotor.isBusy()) {
        }

        //stop
        StopDriving();

        //reset for next go-around
        ResetEncoders();
    }
    public void TurnInPlaceLeft(double power, int degrees){
        degrees*=TURNING_CONSTANT;

        //Set target position
        robot.leftMotor.setTargetPosition(-degrees);
        robot.rightMotor.setTargetPosition(degrees);

        //set drive power
        DriveForward(power);

        //Confirm that motors are done
        while(robot.leftMotor.isBusy() && robot.rightMotor.isBusy()) {
        }

        //stop
        StopDriving();

        //reset for next go-around
        ResetEncoders();
    }
    public void ResetEncoders(){
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void TurnInPlace(double power, int degrees, String direction){
        switch(direction){
            case "left":
                TurnInPlaceLeft(power,degrees);
                break;
            case "right":
                TurnInPlaceRight(power,degrees);
                break;
            default:
                break;
        }
    }

    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        loops=1;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    public int loops=1;

    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());
        // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
        // leftMotor.setPower(-gamepad1.left_stick_y);
        // rightMotor.setPower(-
        /*
        if(gamepad1.right_bumper){
            number++ ;
        }
        if(gamepad1.left_bumper){
            number--;
        }
        telemetry.addData("Count: ", number);
        DriveForwardDistance(.5,number*144) ;
        */
        /*
        switch (loops){
            case 1: case 2:
                DriveForwardTurns(1.0,5);
                break;
            case 3: case 4:
                wait(3000);
                break;
            case 5: case 6: case 7:
                DriveBackwardTurns(0.5,3);
                break;
            case 8:
                DriveBackwardTurns(0.5,1);
                break;
            default:
                break;
        }
        loops++;
        */

        //Do Shooting Stuff (Compensate later)
        if(loops==1)
            DriveForwardDistance(.5,38);
        if(loops==2)
            TurnInPlaceLeft(.7,45);
        if(loops==3)
            DriveForwardDistance(.5,10);
        if(loops==4)
            DriveBackwardDistance(.5,10);
        if(loops==5)
            TurnInPlaceLeft(.7,90);
        if(loops==6)
            DriveForwardDistance(.5,38);
        if(loops==7)
            TurnInPlaceRight(.7,45);
        if(loops==8)
            DriveForwardDistance(.5,15);



        loops++;



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

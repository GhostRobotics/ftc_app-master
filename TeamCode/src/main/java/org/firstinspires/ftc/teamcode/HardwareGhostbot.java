package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.*;

/**
 * Changed by Josh on 4 Jan 2017
 *  This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Ghostbot.
 * See GhostbotTeleopTank_Iterative and others classes starting with "Ghostbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motors:        "left_drive"
 * Motor channel:  Right drive motors:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "launcher
 *
 * Hi, it's Josh again on 2/17/17. I cleaned up old commented-out code across the codebase in
 * preparation for the new robot, so again, if anything breaks, please call or text 978-496-5659.
 */
public class HardwareGhostbot
{
    /* Public OpMode members. */
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public DcMotor launcher;
    public DcMotor lift;
    public DcMotor spinner;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareGhostbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor   = hwMap.dcMotor.get("leftmotor");
        rightMotor  = hwMap.dcMotor.get("rightmotor");
        launcher    = hwMap.dcMotor.get("launcher");
        lift        = hwMap.dcMotor.get("lift");
        spinner     = hwMap.dcMotor.get("spinner");

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        launcher.setPower(0);
        lift.setPower(0);
        spinner.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(RUN_WITHOUT_ENCODER);
        rightMotor.setMode(RUN_WITHOUT_ENCODER);
        launcher.setMode(RUN_WITHOUT_ENCODER);
        lift.setMode(RUN_WITHOUT_ENCODER);
        spinner.setMode(RUN_WITHOUT_ENCODER);

        //set direction
        leftMotor.setDirection(REVERSE);
        rightMotor.setDirection(FORWARD);
        launcher.setDirection(REVERSE);
        lift.setDirection(FORWARD);
        spinner.setDirection(FORWARD); // Because Will said that the connectors were "sketchy"

        launcher.setZeroPowerBehavior(BRAKE);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}


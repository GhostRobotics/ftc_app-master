package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareGhostbot;

/**
 * Created by 7876 on 12/21/2016.
 */

@Autonomous(name="Ghostbot: Auto 2", group="Ghostbot")
public class Auto2 extends OpMode{
    HardwareGhostbot robot = new HardwareGhostbot(); // use the class created to define a Ghostbot's hardware

    @Override
    public void init(){
        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Driver");
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
        driveForward(100);

        delay(2000);

        stopDrive();

        delay(1000);

        driveForward(-100);

        delay(2000);

        stopDrive();

        delay(1000);
    }

    @Override
    public void stop() {
    }

    //helper methods
    private void delay(int mili){   //delays for certain number of miliseconds
        try {
            Thread.sleep(mili);
        }catch(InterruptedException ex){
            robot.leftMotor.setPower(0);
            robot.leftMotor.setPower(0);

            Thread.currentThread().interrupt();
        }
    }
    private void driveForward(int power){   //sets both drive motors to the specified power
        robot.leftMotor.setPower(power);
        robot.rightMotor.setPower(power);
    }
    private void stopDrive(){       //stops driving
        driveForward(0);
    }
}


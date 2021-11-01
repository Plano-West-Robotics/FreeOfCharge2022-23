package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutoModeTest extends LinearOpMode {
    MotorBox motors;


    // part that actually runs
    @Override
    public void runOpMode(){
        initStuff();
        motors.drive(10, 0, 0, 1);
    }


    // converts turns at a specific angle
    // assuming value passed is never 0
    // TODO: measure encoder value corresponding to 90 degrees
    /*
    public void turn(int angle){
        int strength;
        int direction;
        if (angle > 0)
            direction = 1;
        else
            direction = -1;
        strength = // TODO: measure and add corresponding code
        motors.drive(strength, 0, direction, 1);
    }
    */


    // initializes the MotorBox motors
    public void initStuff(){
        motors = new MotorBox(
                hardwareMap.get(DcMotor.class, "frontRight"),
                hardwareMap.get(DcMotor.class, "frontLeft"),
                hardwareMap.get(DcMotor.class, "rearRight"),
                hardwareMap.get(DcMotor.class, "rearLeft"),
                true
        );
    }
}

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class EncoderValueTester extends OpMode {
    MotorBox motors;

    public void loop(){
        if (gamepad1.x)
            motors.drivePositionX(10);
        if (gamepad1.y)
            motors.drivePositionY(10);
        if (gamepad1.a)
            motors.turn(10);
    }

    public void init(){
        motors = new MotorBox(
                hardwareMap.get(DcMotor.class, "frontRight"),
                hardwareMap.get(DcMotor.class, "frontLeft"),
                hardwareMap.get(DcMotor.class, "rearRight"),
                hardwareMap.get(DcMotor.class, "rearLeft"),
                true
        );
    }
}

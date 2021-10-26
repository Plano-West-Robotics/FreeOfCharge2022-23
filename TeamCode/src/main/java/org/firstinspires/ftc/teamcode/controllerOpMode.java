package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class controllerOpMode extends OpMode {
    private DcMotor frontRight, frontLeft, rearRight, rearLeft;
    private double drive,strafe, turn;
    private double speed;
    MotorControl driveMotors;




    public void loop() {

        detectSpeedChange();

        drive = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;

        telemetry.addData("Speed", "Current Speed = " + Math.round(speed*100));
        telemetry.update();
        driveMotors.drive(drive, strafe, turn, speed);

    }

    public void init(){
        driveMotors = new MotorControl();
        speed = 0.25;
    }

    public void detectSpeedChange(){
        if (gamepad1.dpad_up){
            if (speed <= 1) {
                speed += 0.0005;
            }
        }
        if (gamepad1.dpad_down){
            if (speed >= 0) {
                speed -= 0.0005;
            }
        }
    }


}

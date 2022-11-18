package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp
public class ChassisDrive extends OpMode {
    private DcMotor frontRight, frontLeft, rearRight, rearLeft;
    private double frontRightPower, frontLeftPower, rearRightPower, rearLeftPower,
            xDrive, yDrive, turn, speed;

    public void loop() {
        yDrive = gamepad1.left_stick_y;
        xDrive = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;

        detectSpeedChange();
        calcDrive();

        frontRight.setPower(frontRightPower);
        frontLeft.setPower(frontLeftPower);
        rearRight.setPower(rearRightPower);
        rearLeft.setPower(rearLeftPower);

        telemetry.addData("Speed", "Current Speed = " + Math.round(speed * 100));
        telemetry.update();
    }

    public void init() {
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setPower(0);
        frontLeft.setPower(0);
        rearRight.setPower(0);
        rearLeft.setPower(0);

        speed = 1;
    }

    public void detectSpeedChange() {
        if (gamepad1.dpad_up) {
            if (speed <= 1) {
                speed += 0.0005;
            }
        }
        if (gamepad1.dpad_down) {
            if (speed >= 0) {
                speed -= 0.0005;
            }
        }
    }

    public void calcDrive() {
        frontRightPower = yDrive + xDrive;
        frontLeftPower = yDrive - xDrive;
        rearRightPower = yDrive - xDrive;
        rearLeftPower = yDrive + xDrive;

        frontRightPower += turn;
        rearRightPower += turn;
        frontLeftPower -= turn;
        rearLeftPower -= turn;

        frontRightPower *= speed;
        rearRightPower *= speed;
        frontLeftPower *= speed;
        rearLeftPower *= speed;
    }
}

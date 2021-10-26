package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;


import com.qualcomm.robotcore.hardware.DcMotor;


public class MotorControl {
    private DcMotor frontRight, frontLeft, rearRight, rearLeft;
    private double frontRightPower, frontLeftPower, rearRightPower, rearLeftPower;

    public MotorControl(){
        init();
    }
    private void calcDrive(double yDrive, double xDrive){
        frontRightPower = yDrive - xDrive;
        frontLeftPower = yDrive + xDrive;
        rearRightPower = yDrive + xDrive;
        rearLeftPower = yDrive - xDrive;

    }

    private void addTurn(double turn){
        if (turn > 0) {
            frontRightPower *= (1 - turn);
            rearRightPower *= (1 - turn);
        }
        if (turn < 0) {
            frontLeftPower *= (1 + turn);
            rearLeftPower *= (1 + turn);
        }
    }


    public void drive(double yDrive, double xDrive, double speed, double turn){
        calcDrive(yDrive, xDrive);
        if (turn != 0) {
            addTurn(turn);
        }
        frontRightPower*=speed;
        frontLeftPower*=speed;
        rearRightPower*=speed;
        rearLeftPower*=speed;

        frontRight.setPower(-1*frontRightPower);
        frontLeft.setPower(frontLeftPower);
        rearRight.setPower(-1*rearRightPower);
        rearLeft.setPower(rearLeftPower);
    }

    private void init(){
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");

        // stops and resets encoders
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // turns on encoders
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}

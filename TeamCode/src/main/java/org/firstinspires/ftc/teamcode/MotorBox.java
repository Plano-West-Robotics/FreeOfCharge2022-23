package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;



public class MotorBox {
    private DcMotor frontRight, frontLeft, rearRight, rearLeft;
    private double frontRightPower, frontLeftPower, rearRightPower, rearLeftPower;
    private boolean toPosition;

    public MotorBox(DcMotor frontRight, DcMotor frontLeft, DcMotor rearRight, DcMotor rearLeft, boolean toPosition){
        init(frontRight, frontLeft, rearRight, rearLeft, toPosition);
    }

    // drive code
    // works with both encoder power and encoder position theoretically
    public String drive(double yDrive, double xDrive, double turn, double speed) {

        // sets up the initial power values based on input
        frontRightPower = yDrive - xDrive;
        frontLeftPower = yDrive + xDrive;
        rearRightPower = yDrive + xDrive;
        rearLeftPower = yDrive - xDrive;

        // debugging
        String returnVal = "Front Right = " + frontRightPower + "\nFront Left = " + frontLeftPower + "\nRear Right = " + rearRightPower + "\nRear Left = " + rearLeftPower +
                "\nyDrive = " + yDrive + "\nxDrive = " + xDrive +
                "\nTurn = " + turn;

        // does the turn calculations
        if (turn != 0) {
            addTurn(turn);
        }

        // multiplies by speed
        frontRightPower *= speed;
        frontLeftPower *= speed;
        rearRightPower *= speed;
        rearLeftPower *= speed;

        // applies the power
        frontRight.setPower(-1 * frontRightPower);
        frontLeft.setPower(frontLeftPower);
        rearRight.setPower(-1 * rearRightPower);
        rearLeft.setPower(rearLeftPower);

        // return for debugging
        return returnVal;
    }


    // changes value corresponding to turn
    // two states based on if the MotorControl
    private void addTurn(double turn){
        if (toPosition) {
            if (turn > 0) {
                frontRightPower *= -1;
                rearRightPower *= -1;
            }
            if (turn < 0) {
                frontLeftPower *= -1;
                rearLeftPower *= -1;
            }
        }
        else {
            if (turn > 0) {
                frontRightPower *= (1 - turn);
                rearRightPower *= (1 - turn);
            }
            if (turn < 0) {
                frontLeftPower *= (1 - turn);
                rearLeftPower *= (1 - turn);
            }
        }
    }



    // initializes the motors, sets up the correct encoder mode, and sets the toPosition boolean
    private void init(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4, boolean toPosition){
        frontRight = motor1;
        frontLeft = motor2;
        rearRight = motor3;
        rearLeft = motor4;

        // stops and resets encoders
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // sets the toPosition boolean
        this.toPosition = toPosition;

        // turns on encoders
        if (toPosition) {
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else{
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;



public class MotorBox {
    private DcMotor frontRight, frontLeft, rearRight, rearLeft;
    private double frontRightPower, frontLeftPower, rearRightPower, rearLeftPower;

    public MotorBox(DcMotor frontRight, DcMotor frontLeft, DcMotor rearRight, DcMotor rearLeft, boolean toPosition){
        init(frontRight, frontLeft, rearRight, rearLeft, toPosition);
    }

    // drive code
    // works with both encoder power and encoder position theoretically
    public String drivePower(double yDrive, double xDrive, double turn, double speed) {

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

    public int drivePositionY(int distance){
        int numEncoders;

        //code to convert the distance to a number of encoders
        numEncoders = distance;

        frontRight.setTargetPosition(-1 * numEncoders);
        frontLeft.setTargetPosition(numEncoders);
        rearRight.setTargetPosition(-1 * numEncoders);
        rearLeft.setTargetPosition(numEncoders);

        return numEncoders;
    }

    public int drivePositionX(int distance){
        int numEncoders;

        //code to convert distance to a number of encoders
        numEncoders = distance;

        frontRight.setTargetPosition(numEncoders);
        frontLeft.setTargetPosition(numEncoders);
        rearRight.setTargetPosition(-1 * numEncoders);
        rearLeft.setTargetPosition(-1 * numEncoders);

        return numEncoders;
    }


    public int turn(int angle){
        int numEncoders;

        // code to convert angle into a number of encoders
        // will have a negative number for negative angles
        // turning right is positive left is negative
        numEncoders = angle;

        frontRight.setTargetPosition(numEncoders);
        frontLeft.setTargetPosition(numEncoders);
        rearRight.setTargetPosition(numEncoders);
        rearLeft.setTargetPosition(numEncoders);

        return numEncoders;
    }

    // changes value corresponding to turn
    // two states based on if the MotorControl
    private void addTurn(double turn){
        if (turn > 0) {
            frontRightPower *= 2*(0.5 - turn);
            rearRightPower *= 2*(0.5 - turn);
        }
        if (turn < 0) {
            frontLeftPower *= 2*(0.5 - turn);
            rearLeftPower *= 2*(0.5 - turn);
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

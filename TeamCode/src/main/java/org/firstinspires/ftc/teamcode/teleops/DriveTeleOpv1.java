package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp
public class DriveTeleOpv1 extends OpMode {

    //Drive Variables
    private DcMotor motorFR, motorFL, motorRR, motorRL;
    private double powerFR, powerFL, powerRR, powerRL;
    private double drive = 0, strafe = 0, turn = 0;

    @Override
    public void loop() {
        takeControllerInput();

        drive();
    }

    private void takeControllerInput() {
        drive = -1 * gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;

        /*
        if (gamepad2.a) {
            if (!wasPressingA) {

            }
            wasPressingA = true;
        } else wasPressingA = false;

        holdServoPow = gamepad2.left_stick_x;
         */

    }

    private void drive() {

        powerFR = drive - strafe;
        powerFL = drive + strafe;
        powerRR = drive + strafe;
        powerRL = drive - strafe;

        addTurn(turn);

        // multiplies by speed
        double speed = 1;
        powerFR *= speed;
        powerFL *= speed;
        powerRR *= speed;
        powerRL *= speed;

        // applies the power
        motorFR.setPower(powerFR);
        motorFL.setPower(powerFL);
        motorRR.setPower(powerRR);
        motorRL.setPower(powerRL);

        telemetry.addData("powerFR", powerFR);
        telemetry.update();
    }

    private void addTurn(double turn) {
        powerFR -= turn;
        powerRR -= turn;
        powerFL += turn;
        powerRL += turn;
    }

    @Override
    public void init() {
        //initializes the drive motors
        motorFR = hardwareMap.get(DcMotor.class, "frontRight");
        motorFL = hardwareMap.get(DcMotor.class, "frontLeft");
        motorRR = hardwareMap.get(DcMotor.class, "rearRight");
        motorRL = hardwareMap.get(DcMotor.class, "rearLeft");

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRL.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFR.setPower(0);
        motorFL.setPower(0);
        motorRR.setPower(0);
        motorRL.setPower(0);
    }
}

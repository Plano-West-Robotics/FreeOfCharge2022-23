package org.firstinspires.ftc.teamcode.createdcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MonoControllerDriveTeleOp extends OpMode {

    //Drive Variables
    private DcMotor motorFR, motorFL, motorRR, motorRL;
    private double powerFR, powerFL, powerRR, powerRL;
    private DcMotor motorLiftUp;
    private DcMotor motorLiftDown;
    private double powerLiftUp;
    private double powerLiftDown;
    private double drive = 0, strafe = 0, turn = 0;
    private double speed = 1;
    private final boolean lockSpeed = true;

    @Override
    public void loop() {
        takeControllerInput();

        drive();
    }


    private void takeControllerInput() {
        drive = -1 * gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_trigger - gamepad1.left_trigger;
    }


    private void drive() {
        if (gamepad1.left_bumper) speed = Math.max(0, speed - 0.01);
        if (gamepad1.right_bumper) speed = Math.min(1, speed + 0.01);

        powerFR = drive - strafe;
        powerFL = drive + strafe;
        powerRR = drive + strafe;
        powerRL = drive - strafe;

        if (gamepad1.a) {
            powerLiftUp = 0.3;
            powerLiftDown = 0.3;
        } else if (gamepad1.b) {
            powerLiftDown = -0.3;
            powerLiftUp = -0.3;
        } else {
            powerLiftUp = 0;
            powerLiftDown = 0;
        }

        addTurn(turn);

        // multiplies by speed
        powerFR *= speed;
        powerFL *= speed;
        powerRR *= speed;
        powerRL *= speed;

        // applies the power
        motorFR.setPower(powerFR);
        motorFL.setPower(powerFL);
        motorRR.setPower(powerRR);
        motorRL.setPower(powerRL);

        motorLiftUp.setPower(powerLiftUp);
        motorLiftDown.setPower(powerLiftDown);

        telemetry.addData("speed", speed);
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

        motorLiftUp = hardwareMap.get(DcMotor.class, "liftUp");
        motorLiftDown = hardwareMap.get(DcMotor.class, "liftDown");

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
        motorLiftUp.setPower(0);
        motorLiftDown.setPower(0);
    }
}

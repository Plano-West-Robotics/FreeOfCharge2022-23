package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.text.DecimalFormat;

@TeleOp(name = "TELEOP - USE THIS ONE")
public class DriveTeleOp extends OpMode {
    //Drive Variables
    private DcMotor motorFR, motorFL, motorRR, motorRL;
    private double powerFR, powerFL, powerRR, powerRL;
    private DcMotor motorLiftLeft;
    private DcMotor motorLiftRight;
    private double powerLift;
    private Servo claw;
    private double clawPos = 1;
    private double drive = 0, strafe = 0, turn = 0;
    private double speed = 1;
    private boolean last_left_bumper = false;
    private boolean last_right_bumper = false;
    private boolean last_claw_control = false;

    @Override
    public void loop() {
        takeControllerInput();

        drive();
    }


    private void takeControllerInput() {
        drive = -1 * gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;
        powerLift = -0.75 * gamepad2.left_stick_y;
        if (gamepad1.left_bumper && last_left_bumper != gamepad1.left_bumper) speed = Math.max(0.15, speed - 0.15);
        if (gamepad1.right_bumper && last_right_bumper != gamepad1.right_bumper) speed = Math.min(1, speed + 0.15);
        if (gamepad2.a && last_claw_control != gamepad2.a) clawPos = (clawPos == 0 ? 1 : 0);

        last_left_bumper = gamepad1.left_bumper;
        last_right_bumper = gamepad1.right_bumper;
        last_claw_control = gamepad2.a;
    }

    private void drive() {
        powerFR = drive - strafe;
        powerFL = drive + strafe;
        powerRR = drive + strafe;
        powerRL = drive - strafe;

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

        motorLiftLeft.setPower(powerLift);
        motorLiftRight.setPower(powerLift);

        // TODO: keep / remove these lines based on empirical testing
        claw.setPosition(clawPos);

        DecimalFormat df = new DecimalFormat("#%");
        telemetry.addData("speed", df.format(speed));
        telemetry.addData("clawPos", df.format(clawPos));
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

        motorLiftLeft = hardwareMap.get(DcMotor.class, "liftLeft");
        motorLiftRight = hardwareMap.get(DcMotor.class, "liftRight");

        claw = hardwareMap.get(Servo.class, "claw");

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
        motorLiftLeft.setPower(0);
        motorLiftRight.setPower(0);
    }
}
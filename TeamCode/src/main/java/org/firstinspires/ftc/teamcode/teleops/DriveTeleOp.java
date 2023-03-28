package org.firstinspires.ftc.teamcode.teleops;
//imports 
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.text.DecimalFormat;

/**
* DriveTeleOps utilizes inheritance by extending to 
* OpMode. Initalizing variables for the drive constants. 
*/ 
@TeleOp(name = "TELEOP - USE THIS ONE")
public class  DriveTeleOp extends OpMode {
    //Drive Variables
    private DcMotor motorFR, motorFL, motorBR, motorBL;
    private double powerFR, powerFL, powerBR, powerBL;
    private DcMotor motorLift;
    private double powerLift;
    private Servo claw;
    private double clawPos = 1;
    private double drive = 0, strafe = 0, turn = 0;
    private double speed = 0.4;
    private boolean last_left_bumper = false;
    private boolean last_right_bumper = false;
    private boolean last_claw_control = false;

    @Override
    public void loop() {
        takeControllerInput();

        drive();
    }

       /**
       * Utilizes inout given from the controller. 
       */ 
    private void takeControllerInput() {
        drive = -1 * gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;
        powerLift = gamepad2.left_stick_y;
        if (gamepad1.left_bumper && last_left_bumper != gamepad1.left_bumper) speed = Math.max(0.15, speed - 0.15);
        if (gamepad1.right_bumper && last_right_bumper != gamepad1.right_bumper) speed = Math.min(1, speed + 0.15);
        if (gamepad2.a && last_claw_control != gamepad2.a) clawPos = (clawPos == 0 ? 1 : 0);

        if (motorLift.getCurrentPosition() > -10) powerLift = Range.clip(powerLift, -1, 0);

        last_left_bumper = gamepad1.left_bumper;
        last_right_bumper = gamepad1.right_bumper;
        last_claw_control = gamepad2.a;
    }

    private void drive() {
        powerFR = drive - strafe;
        powerFL = drive + strafe;
        powerBR = drive + strafe;
        powerBL = drive - strafe;

        addTurn(turn);

        // multiplies by speed
        powerFR *= speed;
        powerFL *= speed;
        powerBR *= speed;
        powerBL *= speed;

        // applies the power
        motorFR.setPower(powerFR);
        motorFL.setPower(powerFL);
        motorBR.setPower(powerBR);
        motorBL.setPower(powerBL);

        motorLift.setPower(powerLift);

        // TODO: keep / remove these lines based on empirical testing
        claw.setPosition(clawPos);

        DecimalFormat df = new DecimalFormat("#%");
        telemetry.addData("speed", df.format(speed));
        telemetry.addData("clawPos", df.format(clawPos));
        telemetry.update();
    }

    private void addTurn(double turn) {
        powerFR -= turn;
        powerBR -= turn;
        powerFL += turn;
        powerBL += turn;
    }

    @Override
    public void init() {
        //initializes the drive motors
        motorFR = hardwareMap.get(DcMotor.class, "frontRight");
        motorFL = hardwareMap.get(DcMotor.class, "frontLeft");
        motorBR = hardwareMap.get(DcMotor.class, "rearRight");
        motorBL = hardwareMap.get(DcMotor.class, "rearLeft");

        motorLift = hardwareMap.get(DcMotor.class, "slide");
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        claw = hardwareMap.get(Servo.class, "claw");
        // only allow claw to open partially
        claw.scaleRange(0, 0.65);

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);
        motorLift.setPower(0);
    }
}

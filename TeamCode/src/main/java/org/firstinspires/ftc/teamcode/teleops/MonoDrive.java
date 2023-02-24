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
@TeleOp
public class MonoDrive extends OpMode {
    //Drive Variables
    private DcMotor motorFR, motorFL, motorRR, motorRL;
    private double powerFR, powerFL, powerRR, powerRL;
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
        turn = gamepad1.right_trigger - gamepad1.left_trigger;
        powerLift = gamepad1.right_stick_y;
        if (gamepad1.left_bumper && last_left_bumper != gamepad1.left_bumper) speed = Math.max(0.15, speed - 0.15);
        if (gamepad1.right_bumper && last_right_bumper != gamepad1.right_bumper) speed = Math.min(1, speed + 0.15);
        if (gamepad1.a && last_claw_control != gamepad1.a) clawPos = (clawPos == 0 ? 1 : 0);

        if (motorLift.getCurrentPosition() > -10) powerLift = Range.clip(powerLift, -1, 0);

        last_left_bumper = gamepad1.left_bumper;
        last_right_bumper = gamepad1.right_bumper;
        last_claw_control = gamepad1.a;
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

        motorLift = hardwareMap.get(DcMotor.class, "slide");
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        claw = hardwareMap.get(Servo.class, "claw");
        // only allow claw to move hal
        claw.scaleRange(0, 0.75);

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
        motorLift.setPower(0);
    }
}

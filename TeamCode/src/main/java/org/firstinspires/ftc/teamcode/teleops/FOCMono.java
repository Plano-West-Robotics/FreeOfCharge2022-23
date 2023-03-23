package org.firstinspires.ftc.teamcode.teleops;
//imports 
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.text.DecimalFormat;

/**
* DriveTeleOps utilizes inheritance by extending to 
* OpMode. Initalizing variables for the drive constants. 
*/
@TeleOp(name = "Field Oriented Mono")
public class FOCMono extends OpMode {
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
    private IMU imu;

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
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = drive * Math.cos(heading) - strafe * Math.sin(heading);
        double rotY = drive * Math.sin(heading) + strafe * Math.cos(heading);
        powerFR = rotX - rotY;
        powerFL = rotX + rotY;
        powerBR = rotX + rotY;
        powerBL = rotX - rotY;

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
        telemetry.addData("heading", Math.toDegrees(heading));
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

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                // TODO: change these parameters if they are not accurate
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));
    }
}

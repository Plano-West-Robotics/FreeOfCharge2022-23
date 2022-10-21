package org.firstinspires.ftc.teamcode.createdcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DriveTeleOpv2 extends OpMode {

    //Drive Variables
    private DcMotor motorFR, motorFL, motorRR, motorRL;
    private double powerFR, powerFL, powerRR, powerRL;
    private double drive = 0, strafe = 0, turn = 0;
    private double speed = 1;
    private boolean lockSpeed = true;
    private double constantSpeedMult = 0.5;
    private final double constantSpeedMultChangeMult = 0.25;
    private boolean wasPressingDpadUp = false, wasPressingDpadDown = false;

    //carousel servo things
    private DcMotor carouselServo1, carouselServo2;
    private boolean turnCarouselRight = false, turnCarouselLeft = false;
    private final double carouselPower = 0.5;

    //Arm Variables
    private DcMotor armOne, armTwo;
    private double armPow = 0.5;
    private final double armTwoPos = 0;
    private final double armSpeedMod = 0.5;
    private Servo grabServo;
    private double grabServoPos;
    private double holdServoPow;
    private final double servoSpeedMod = 0.01;
    private boolean raiseArm;
    private boolean wasPressingA, wasPressingB, wasPressingX, wasPressingY;

    @Override
    public void loop() {
        takeControllerInput();

        drive();
        moveArm();
        armGrab();
        checkCarousel();

        telemetry.update();
    }

    private void takeControllerInput() {
        drive = -1 * gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;

        if (gamepad1.a) {
            if (!wasPressingA) {
                lockSpeed = !lockSpeed;
            }
            wasPressingA = true;
        } else wasPressingA = false;


        if (gamepad1.dpad_up) {
            if (!wasPressingDpadUp) {
                constantSpeedMult = Math.min(constantSpeedMult + constantSpeedMultChangeMult, 1);
            }
            wasPressingDpadUp = true;
        } else wasPressingDpadUp = false;

        if (gamepad1.dpad_down) {
            if (!wasPressingDpadDown) {
                constantSpeedMult = Math.max(constantSpeedMult - constantSpeedMultChangeMult, 0);
            }
            wasPressingDpadDown = true;
        } else wasPressingDpadDown = false;

        telemetry.addData("Constant Speed Mult", constantSpeedMult);

        speed = lockSpeed ? constantSpeedMult : gamepad1.right_trigger;

        turnCarouselRight = gamepad2.right_bumper;
        turnCarouselLeft = gamepad2.left_bumper;

        armPow = gamepad2.right_stick_y;

        grabServoPos = Math.min(Math.max(grabServoPos - gamepad2.left_stick_y * servoSpeedMod, 0.33), 0.6);


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

        telemetry.addData("powerFR", powerFR);

    }

    private void addTurn(double turn) {
        powerFR -= turn;
        powerRR -= turn;
        powerFL += turn;
        powerRL += turn;
    }


    private void moveArm() {

        armOne.setPower(armPow * armSpeedMod);
        armTwo.setPower(armPow * armSpeedMod);
        telemetry.addData("Arm One Position", armOne.getCurrentPosition());
    }

    private void armGrab() {
        grabServo.setPosition(grabServoPos);
        telemetry.addData("Servo Position", grabServoPos);
    }

    private void checkCarousel() {
        if (turnCarouselRight) {
            carouselServo1.setPower(carouselPower);
            carouselServo2.setPower(carouselPower);
        } else if (turnCarouselLeft) {
            carouselServo1.setPower(carouselPower);
            carouselServo2.setPower(carouselPower);
        } else {
            carouselServo1.setPower(0);
            carouselServo2.setPower(0);
        }
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

        //initializes the arm motors and servos
        armOne = hardwareMap.get(DcMotor.class, "armOne");
        armTwo = hardwareMap.get(DcMotor.class, "armTwo");
        armOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armOne.setDirection(DcMotorSimple.Direction.REVERSE);
        armOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        grabServo = hardwareMap.get(Servo.class, "grabServo");
        grabServo.setPosition(0.6);


        //initializes the carousel servo
        carouselServo1 = hardwareMap.get(DcMotor.class, "spinnyBoyOne");
        carouselServo2 = hardwareMap.get(DcMotor.class, "spinnyBoyTwo");
    }
}

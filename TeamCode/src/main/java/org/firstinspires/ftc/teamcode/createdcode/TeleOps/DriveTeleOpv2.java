package org.firstinspires.ftc.teamcode.createdcode.TeleOps;

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

    //carousel servo things
    private CRServo carouselServo;
    private boolean turnCarouselRight = false, turnCarouselLeft = false;;
    private double carouselPower = 1;

    //Arm Variables
    private DcMotor armOne, armTwo;
    private double armPow = 0.5;
    private double armTwoPos = 0;
    private double armSpeedMod = 0.5;
    private Servo grabServo;
    private double grabServoPos, holdServoPow, servoSpeedMod = 0.05;
    private boolean raiseArm;
    private boolean wasPressingA, wasPressingB, wasPressingX, wasPressingY;

    @Override
    public void loop() {
        takeControllerInput();

        drive();
        moveArm();
        armGrab();
        checkCarousel();
    }

    private void takeControllerInput(){
        drive = -1*gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;

        if (gamepad1.a) {
            if (!wasPressingA) {
                lockSpeed = !lockSpeed;
            }
            wasPressingA = true;
        }
        else wasPressingA = false;
        speed = lockSpeed ? 1 : gamepad1.right_trigger;

        turnCarouselRight = gamepad2.right_bumper;
        turnCarouselLeft = gamepad2.left_bumper;


        armPow = gamepad2.right_stick_y;

        grabServoPos = Math.min(Math.max(grabServoPos - gamepad2.left_stick_y * servoSpeedMod, 0.0),0.4);


    }


    private void drive(){

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
        telemetry.update();
    }

    private void addTurn(double turn){
        powerFR -= turn;
        powerRR -= turn;
        powerFL += turn;
        powerRL += turn;
    }


    private void moveArm(){

        armOne.setPower(armPow * armSpeedMod);
        armTwo.setPower(armPow * armSpeedMod);
    }

    private void armGrab(){
        grabServo.setPosition(grabServoPos);
    }

    private void checkCarousel() {
        if (turnCarouselRight)
            carouselServo.setPower(carouselPower);
        else if (turnCarouselLeft)
            carouselServo.setPower(-1*carouselPower);
        else
            carouselServo.setPower(0);
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
        armTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        armTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        grabServo = hardwareMap.get(Servo.class, "grabServo");
        grabServo.setPosition(1);


        //initializes the carousel servo
        carouselServo = hardwareMap.get(CRServo.class, "spinnyBoy");
    }
}

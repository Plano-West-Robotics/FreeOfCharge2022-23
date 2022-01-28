package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class DriveTeleOpv1 extends OpMode {

    //Drive Variables
    private DcMotor motorFR, motorFL, motorRR, motorRL;
    private double powerFR, powerFL, powerRR, powerRL;
    private double drive = 0, strafe = 0, turn = 0;
    private double speed = 1;

    //carousel servo things
    private CRServo carouselServo;
    private boolean turnCarouselRight = false, turnCarouselLeft = false;;
    private double carouselPower = 0.5;

    //Arm Variables
    private DcMotor armOne, armTwo;
    private double armOnePow = 0.5, armTwoPow = 0.5;
    private double armTwoPos = 0;
    private double armSpeedMod = 0.5;
    private Servo grabServo, holdServo;
    private double grabServoPow, holdServoPow, servoSpeedMod = 0.05;
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

        armOnePow = gamepad2.right_stick_x;
        turnCarouselRight = gamepad2.b;
        turnCarouselLeft = gamepad2.x;



        if (gamepad2.a) {
            if (!wasPressingA) {
                raiseArm = !raiseArm;
            }
            wasPressingA = true;
        }
        else wasPressingA = false;

        armTwoPow = raiseArm ? 0.25 : gamepad2.right_stick_y * 0.25;

        grabServoPow = gamepad2.left_stick_y;
        holdServoPow = gamepad2.left_stick_x;

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
        armOne.setPower(armOnePow * armSpeedMod);

        armTwo.setPower(armTwoPow);
    }

    private void armGrab(){
        grabServo.setPosition(grabServo.getPosition() + -1*grabServoPow*servoSpeedMod);
        holdServo.setPosition(holdServo.getPosition() + -1*holdServoPow*servoSpeedMod);
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
        armTwo.setDirection(DcMotorSimple.Direction.FORWARD);
        armTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        raiseArm = true;

        grabServo = hardwareMap.get(Servo.class, "grabServo");
        holdServo = hardwareMap.get(Servo.class, "holdServo");
        grabServo.setPosition(1);
        holdServo.setPosition(0);

        //initializes the carousel servo
        carouselServo = hardwareMap.get(CRServo.class, "duckArm");
    }
}

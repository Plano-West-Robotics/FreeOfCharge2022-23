package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class controllerOpMode extends OpMode {
    private DcMotor spool, spinnyBoy, intakeMotor;
    private Servo bucketServo1, bucketServo2, shippingServo, intakeServo;
    private double drive,strafe, turn, linearSpeed;
    private double speed;
    MotorBox driveMotors;
    double bucketServoPos, shippingServoPos, intakeServoPos;


    // part that actually runs
    // naturally loops
    public void loop() {

        // checks to see if the speed change button is pressed
        detectSpeedChange();
        detectSlideSpeedChange();


        // sets the driver strafe and turn values
        // drive is negated so 1 is forward
        drive = -1*gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;

        // prints the speed
        telemetry.addData("Speed", "Current Speed = " + Math.round(speed*100));
        telemetry.addData("Linear Slide SPeed", "Current Speed = " + Math.round(linearSpeed*100));

        // makes it drive
        // result is just debug values
        String result = driveMotors.drivePower(drive, strafe, turn, speed);
        spool.setPower(-1*gamepad2.left_stick_y*linearSpeed);

        // carousel spinner
        // spins for red on gamepad2's b
        if(gamepad2.b){
            spinnyBoy.setPower(1);
        }
        else {
            spinnyBoy.setPower(0);
        }
        // spins for blue on gamepad2's x
        if(gamepad2.x){
            spinnyBoy.setPower(-1);
        }
        else {
            spinnyBoy.setPower(0);
        }

        // intakeMotor spinner
        // spins on gamepad2's a
        if(gamepad2.a){
            intakeMotor.setPower(0.5);
        }
        else {
            intakeMotor.setPower(0);
        }
        if(gamepad2.y){
            intakeMotor.setPower(-0.5);
        }
        else {
            intakeMotor.setPower(0);
        }
        // changes the bucketServo's position using the gamepad2's right stick y
        bucketServoPositionChange();

        // changes the shippingServo's position using the gamepad2's right and left bumpers
        //shippingServoPositionChange();

        intakeServoPosition();


        //debugging
        //telemetry.addData("Results", result);
        /*
        telemetry.addData("Drive", "Drive = " + drive);
        telemetry.addData("Strafe", "Strafe = " + strafe);
        */

        //show telemetry
        telemetry.update();
    }

    // initializes the driveMotors MotorBox and speed
    public void init(){
        driveMotors = new MotorBox(
                hardwareMap.get(DcMotor.class, "frontRight"),
                hardwareMap.get(DcMotor.class, "frontLeft"),
                hardwareMap.get(DcMotor.class, "rearRight"),
                hardwareMap.get(DcMotor.class, "rearLeft"),
                false
        );
        speed = 0.25;
        linearSpeed = 0.75;
        bucketServoPos = 1;
        shippingServoPos = 0;
        intakeServoPos = 0.2;

        spool = hardwareMap.get(DcMotor.class, "spool");
        spinnyBoy = hardwareMap.get(DcMotor.class, "spinnyBoy");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        bucketServo1 = hardwareMap.get(Servo.class, "bucketServo1");
        bucketServo2 = hardwareMap.get(Servo.class, "bucketServo2");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        //shippingServo = hardwareMap.get(Servo.class, "shippingServo");

        bucketServo1.setDirection(Servo.Direction.REVERSE);

        //bucketServo1.setPosition(bucketServoPos);
        //bucketServo2.setPosition(bucketServoPos);
        intakeServo.setPosition(intakeServoPos);

        spool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // detects the speed change button and changes speed accordingly
    public void detectSpeedChange(){
        if (gamepad1.dpad_up){
            if (speed <= 1) {
                speed += 0.0005;
            }
        }
        if (gamepad1.dpad_down){
            if (speed >= 0) {
                speed -= 0.0005;
            }
        }
    }
    public void detectSlideSpeedChange(){
        if (gamepad2.dpad_up){
            if (linearSpeed <= 1) {
                linearSpeed += 0.0005;
            }
        }
        if (gamepad2.dpad_down){
            if (linearSpeed >= 0) {
                linearSpeed  -= 0.0005;
            }
        }
    }
    public void bucketServoPositionChange(){
        bucketServoPos += 0.001 * gamepad2.right_stick_y;
        if (bucketServoPos > 1){
            bucketServoPos = 1;
        }
        if (bucketServoPos < 0){
            bucketServoPos = 0;
        }
        bucketServo1.setPosition(bucketServoPos);
        bucketServo2.setPosition(bucketServoPos);

    }
    public void intakeServoPosition(){
        if (gamepad2.right_bumper){
            if (intakeServoPos <= 1) {
                intakeServoPos += 0.0005;
            }
        }
        if (gamepad2.left_bumper){
            if (intakeServoPos >= 0) {
                intakeServoPos  -= 0.0005;
            }
        }
        intakeServo.setPosition(intakeServoPos);
    }

    /*
    public void shippingServoPositionChange(){
        if (gamepad2.right_bumper){
            if (shippingServoPos <= 1) {
                shippingServoPos += 0.0005;
            }
        }
        if (gamepad2.left_bumper){
            if (shippingServoPos >= 0) {
                shippingServoPos  -= 0.0005;
            }
        }
        shippingServo.setPosition(shippingServoPos);
    }

     */
}

package org.firstinspires.ftc.teamcode.createdcode.oldthings.autoparking;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.WHEEL_RADIUS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled

@Autonomous(group = "parking")
public class BlueSideParkingAuto extends LinearOpMode {
    //change values
    private static final double TICKS_PER_INCH = (TICKS_PER_REV / GEAR_RATIO) / (WHEEL_RADIUS * 2 * Math.PI);// counts per revolution
    DcMotor frontLeft, rearLeft, rearRight, frontRight, spool, intakeMotor, spinnyBoy;
    Servo bucketServo;


    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        spool = hardwareMap.get(DcMotor.class, "spool");
        spinnyBoy = hardwareMap.get(DcMotor.class, "spinnyBoy");

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setTargetPosition(0);
        frontRight.setTargetPosition(0);
        rearLeft.setTargetPosition(0);
        rearRight.setTargetPosition(0);

        resetMotorDirections();

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        frontRight.setTargetPosition(frontRight.getCurrentPosition() + 500);
        rearRight.setTargetPosition(rearRight.getCurrentPosition() + 500);
        setSpeed(0.2);
        while (rearRight.isBusy()) {
            telemetry.addData("frontRightEncoder", frontRight.getCurrentPosition());
            telemetry.addData("frontLeftEncoder", frontLeft.getCurrentPosition());
            telemetry.addData("rearRightEncoder", rearRight.getCurrentPosition());
            telemetry.addData("rearLeftEncoder", rearLeft.getCurrentPosition());
            telemetry.update();
        }
        setSpeed(0);

        driveForward(-16, 0.2);

        setSpeed(0);

        sleep(4000);

        driveForward(200, .8);

        setSpeed(0);



        /*
        frontRight.setTargetPosition(frontRight.getCurrentPosition()+250);
        rearRight.setTargetPosition(rearRight.getCurrentPosition()+250);
        setSpeed(0.2);
        while (frontRight.isBusy()){
            telemetry.addData("frontRightEncoder", frontRight.getCurrentPosition());
            telemetry.addData("frontLeftEncoder", frontLeft.getCurrentPosition());
            telemetry.addData("rearRightEncoder", rearRight.getCurrentPosition());
            telemetry.addData("rearLeftEncoder", rearLeft.getCurrentPosition());
            telemetry.update();
        }
        setSpeed(0);

        driveForward(100,0.2);
        setSpeed(0);

         */

    }

    public void driveForward(int distance, double speed) {
        int encodersToTurn = (int) Math.round(distance * TICKS_PER_INCH * 0.7);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + encodersToTurn);
        rearLeft.setTargetPosition(rearLeft.getCurrentPosition() + encodersToTurn);

        frontRight.setTargetPosition(frontRight.getCurrentPosition() + encodersToTurn);
        rearRight.setTargetPosition(rearRight.getCurrentPosition() + encodersToTurn);

        setSpeed(speed);

        while (frontLeft.isBusy()) {
            telemetry.addData("frontRightEncoder", frontRight.getCurrentPosition());
            telemetry.addData("frontLeftEncoder", frontLeft.getCurrentPosition());
            telemetry.addData("rearRightEncoder", rearRight.getCurrentPosition());
            telemetry.addData("rearLeftEncoder", rearLeft.getCurrentPosition());
            telemetry.update();
        }


    }


    public void driveBackward(int distance, double speed) {
        distance = (int) (Math.round(distance * TICKS_PER_INCH * 0.7));

        //motor 0 - FL, motor 1 - BL, motor 2 - FR, motor 3 - BR
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + distance);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - distance);
        rearLeft.setTargetPosition(rearLeft.getCurrentPosition() + distance);
        rearRight.setTargetPosition(rearRight.getCurrentPosition() - distance);
        setSpeed(speed);
    }

    public void strafeLeft(int distance, double speed) {
        distance = (int) (Math.round(distance * TICKS_PER_INCH * 0.7));

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - distance);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + distance);
        rearLeft.setTargetPosition(rearLeft.getCurrentPosition() + distance);
        rearRight.setTargetPosition(rearRight.getCurrentPosition() - distance);
        setSpeed(speed);
    }

    public void strafeRight(int distance, double speed) {
        distance = (int) (Math.round(distance * TICKS_PER_INCH * 0.7));

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + distance);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - distance);
        rearLeft.setTargetPosition(rearLeft.getCurrentPosition() - distance);
        rearRight.setTargetPosition(rearRight.getCurrentPosition() + distance);
        setSpeed(speed);
    }
    /*
    public void linearSlideExtendLevel1(){
        motorLinearSlide.setPower(-1.0);
        sleep(1000);
        motorLinearSlide.setPower(1.0);
        sleep(1000);
        motorLinearSlide.setPower(0.0);
        rotateClawServo.setPosition(1);
        clawServo.setPosition(1.0);
    }
    */

    public void setSpeed(double speed) {

        frontLeft.setPower(speed);
        rearLeft.setPower(speed);
        frontLeft.setPower(speed);
        rearRight.setPower(speed);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void rotate90() {
        frontLeft.setPower(-1);
        rearLeft.setPower(-1);
        frontRight.setPower(1);
        rearRight.setPower(1);

        setSpeed(0);
    }

    public void turnRight(int degrees, double speed) {
        double turnDist = 10;


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRight.setDirection(DcMotorSimple.Direction.FORWARD);

        setSpeed(speed);


        setSpeed(0);

    }


    public void turnLeft(int degrees, double speed) {
        frontLeft.setPower(speed);
        rearLeft.setPower(speed);
        frontRight.setPower(speed);
        rearRight.setPower(speed);

        setSpeed(0);

    }

    public void resetMotorDirections() {
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}
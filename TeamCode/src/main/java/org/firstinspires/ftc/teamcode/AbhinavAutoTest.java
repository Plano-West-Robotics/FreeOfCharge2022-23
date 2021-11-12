
package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@Autonomous
public class AbhinavAutoTest extends LinearOpMode{
    //change values
    public static final double REV_COUNT= 1120;//counts per revolution
    public static final double DIAMETER = 4;
    public static final double GEAR_RATIO = 5;
    private static final double INCHES_ROTATION = (REV_COUNT * GEAR_RATIO) / (DIAMETER * Math.PI);// counts per revolution
    DcMotor frontLeft, rearLeft, rearRight, frontRight, spool, intakeMotor;
    Servo rotateClawServo , clawServo;

    Orientation angles = new Orientation();
    double finalAngle;
    Acceleration gravity;

    public void runOpMode() throws InterruptedException{

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        rearLeft = hardwareMap.get(DcMotor.class,"rearLeft");
        rearRight = hardwareMap.get(DcMotor.class,"rearRight");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        spool = hardwareMap.get(DcMotor.class, "spool");

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setTargetPosition(0);
        frontRight.setTargetPosition(0);
        rearLeft.setTargetPosition(0);
        rearRight.setTargetPosition(0);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();


        driveForward(5,0.5);

        //sleep(1000);

        /*
        setSpeed(0);

        driveBackward(5,0.5);

        sleep(1000);

        setSpeed(0);

        strafeLeft(5,0.5);

        sleep(1000);

        setSpeed(0);

        strafeRight(5,0.5);

        sleep(1000);

        setSpeed(0);

        while(opModeIsActive()){

            // carouselPosition();

            //sleep(1000);


        }
        */
    }

    public void driveForward(int distance, double speed){
        distance = (int) Math.round(distance * INCHES_ROTATION);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition()+distance);
        rearLeft.setTargetPosition(rearLeft.getCurrentPosition()+distance);

        frontRight.setTargetPosition(frontRight.getCurrentPosition()+distance);
        rearRight.setTargetPosition(rearRight.getCurrentPosition()+distance);


        setSpeed(speed);

    }
    public void driveBackward(int distance, double speed){
        distance = (int)(Math.round(distance * INCHES_ROTATION * 0.7));

        //motor 0 - FL, motor 1 - BL, motor 2 - FR, motor 3 - BR
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition()+distance);
        frontRight.setTargetPosition(frontRight.getCurrentPosition()-distance);
        rearLeft.setTargetPosition(rearLeft.getCurrentPosition()+distance);
        rearRight.setTargetPosition(rearRight.getCurrentPosition()-distance);
        setSpeed(speed);
    }
    public void strafeLeft(int distance, double speed){
        distance = (int)(Math.round(distance * INCHES_ROTATION * 0.7));

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition()-distance);
        frontRight.setTargetPosition(frontRight.getCurrentPosition()+distance);
        rearLeft.setTargetPosition(rearLeft.getCurrentPosition()+distance);
        rearRight.setTargetPosition(rearRight.getCurrentPosition()-distance);
        setSpeed(speed);
    }
    public void strafeRight(int distance, double speed){
        distance = (int)(Math.round(distance * INCHES_ROTATION * 0.7));

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition()+distance);
        frontRight.setTargetPosition(frontRight.getCurrentPosition()-distance);
        rearLeft.setTargetPosition(rearLeft.getCurrentPosition()-distance);
        rearRight.setTargetPosition(rearRight.getCurrentPosition()+distance);
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

    public void carouselPosition(){
        turn(180,1);
        //sleep(1000);
        spool.setPower(1);
        //sleep(1000);
        rotateClawServo.setPosition(0.0);
        //sleep(1000);
        clawServo.setPosition(1.0);
        //sleep(1000);
        turn(-90,1);
        //sleep(1000);
        driveForward(15,1);
        //sleep(1000);
        intakeMotor.setPower(1);
        turn(-90,1000);
        driveForward(2,1);
        //sleep(1000);
        turn(-90,1000);
        //sleep(1000);
        driveForward(30,1);
        ///sleep(1000);
        setSpeed(0);
        rotate90();
        setSpeed(0);

    }
    public void setSpeed(double speed){

        frontLeft.setPower(speed);
        rearLeft.setPower(speed);
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double currAngle(){

        // Orientation angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES);
        double deltaAngle = angles.thirdAngle - angles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        finalAngle += deltaAngle;
        //angles = angle;
        return finalAngle;

    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public void rotate90(){
        frontLeft.setPower(-1);
        rearLeft.setPower(-1);
        frontRight.setPower(1);
        rearRight.setPower(1);

        setSpeed(0);
    }
    public void turn(int degrees, double speed){
        double angleTurn = currAngle();
        if(angleTurn!=degrees && angleTurn>0  ){
            frontLeft.setPower(speed);
            rearLeft.setPower(speed);
            frontRight.setPower(speed);
            rearRight.setPower(speed);
        }else if(angleTurn!=degrees && angleTurn<0){
            frontLeft.setPower(-speed);
            rearLeft.setPower(-speed);
            frontRight.setPower(speed);
            rearRight.setPower(speed);
        }
        setSpeed(0);
    }
}
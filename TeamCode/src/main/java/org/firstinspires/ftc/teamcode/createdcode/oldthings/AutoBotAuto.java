package org.firstinspires.ftc.teamcode.createdcode.oldthings;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled

@Autonomous
public class AutoBotAuto extends LinearOpMode {
    DcMotor frontRight, frontLeft, rearRight, rearLeft;
    public static int dist = 2000;


    public void runOpMode() {
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontRight.setTargetPosition(dist);
        frontLeft.setTargetPosition(dist);
        rearRight.setTargetPosition(dist);
        rearLeft.setTargetPosition(dist);

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (frontLeft.isBusy() || frontRight.isBusy() || rearLeft.isBusy() || rearRight.isBusy()) {
            double power = 1 - Math.max(Math.min((dist - frontLeft.getCurrentPosition()) / 2000, 1), 0.8);
            frontRight.setPower(power);
            frontLeft.setPower(power);
            rearRight.setPower(power);
            rearLeft.setPower(power);
        }

        motorsPower(0, 0, 0, 0);

        telemetry.addData("Ran", "Ran");
        telemetry.update();
    }

    public void motorsPower(double frontRightPow, double frontLeftPow, double rearRightPow, double rearLeftPow) {
        frontRight.setPower(frontRightPow);
        frontLeft.setPower(frontLeftPow);
        rearRight.setPower(rearRightPow);
        rearLeft.setPower(rearRightPow);

    }


}

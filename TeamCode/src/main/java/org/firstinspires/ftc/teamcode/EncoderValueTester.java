package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class EncoderValueTester extends LinearOpMode {
    MotorBox motors;
    DcMotor frontRight, frontLeft, rearRight, rearLeft;


    public void runOpMode(){
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");

        waitForStart();
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setTargetPosition(200);
        frontLeft.setTargetPosition(200);
        rearRight.setTargetPosition(200);
        rearLeft.setTargetPosition(200);

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRight.setPower(.5);
        frontLeft.setPower(.5);
        rearRight.setPower(.5);
        rearLeft.setPower(.5);
        while (frontLeft.isBusy() || frontRight.isBusy() || rearLeft.isBusy() || rearRight.isBusy()){
            sleep(1);
        }
        telemetry.addData("Ran", "Ran");
        telemetry.update();
    }





}

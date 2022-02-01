package org.firstinspires.ftc.teamcode.createdcode.oldThings;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Autonomous
public class EncoderValueTester extends LinearOpMode {
    DcMotor testMotor;


    public void runOpMode(){
        testMotor = hardwareMap.get(DcMotor.class, "testMotor");
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        testMotor.setTargetPosition(testMotor.getCurrentPosition() + 1120);
        testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        testMotor.setPower(0.2);
        while (testMotor.isBusy()){
        }
        testMotor.setPower(0);
        sleep(500);
    }




}

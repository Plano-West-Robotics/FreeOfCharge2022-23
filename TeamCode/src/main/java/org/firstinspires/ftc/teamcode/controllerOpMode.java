package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class controllerOpMode extends OpMode {
    private DcMotor frontRight, frontLeft, rearRight, rearLeft;
    private double drive,strafe;
    private double driveMult = 0.25;
    public void init(){
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    public void loop() {
        strafe = gamepad1.left_stick_x;
        drive = gamepad1.left_stick_y;

        if (gamepad1.dpad_up){
            if (driveMult<=1) {
                driveMult += 0.0005;
            }
        }
        if (gamepad1.dpad_down){
            if (driveMult>=0) {
                driveMult -= 0.0005;
            }
        }

        telemetry.addData("Speed", "Current Speed = " + Math.round(driveMult*100));
        telemetry.update();


        frontRight.setPower(driveMult*(drive - strafe));
        frontLeft.setPower(-1*driveMult*(drive + strafe));
        rearRight.setPower(driveMult*(drive + strafe));
        rearLeft.setPower(-1*driveMult*(drive - strafe));
    }



}

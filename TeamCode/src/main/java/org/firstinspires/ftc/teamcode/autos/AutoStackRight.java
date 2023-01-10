package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SlidePresets;

@Autonomous
public class AutoStackRight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // set up the API
        API api = new API(this);
        // set up InchWorm
        InchWorm inchWorm = new InchWorm(this);
        inchWorm.setDebug(true);
        // disabled for now because slide is broken
        DcMotor lift = hardwareMap.get(DcMotor.class, "slide");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        Servo claw = hardwareMap.get(Servo.class, "claw");
        claw.scaleRange(0, 0.5);
        claw.setPosition(0);

        api.waitForStart();

        // move lift up to prevent cone from scraping on the floor
        lift.setTargetPosition(SlidePresets.LOW.position);
        lift.setPower(0.75);
        // move forward to prevent scraping against the wall
        inchWorm.setSpeedMultiplier(0.1);
        inchWorm.drive(3.5);

        inchWorm.setSpeedMultiplier(0.25);
        inchWorm.strafe(-15);

        inchWorm.setSpeedMultiplier(0.75);
        inchWorm.drive(25);

        lift.setTargetPosition(SlidePresets.GROUND.position);
        while (lift.isBusy()) {}
        claw.setPosition(1);

        inchWorm.drive(-2.25);

        inchWorm.setSpeedMultiplier(0.25);
        inchWorm.strafe(-11);

        inchWorm.setSpeedMultiplier(0.75);
        inchWorm.drive(24);

        lift.setTargetPosition(SlidePresets.STACK_FIVE.position);

        inchWorm.setSpeedMultiplier(0.1);
        inchWorm.strafe(1);

        inchWorm.setSpeedMultiplier(0.75);
        inchWorm.turnDegrees(-90);

        inchWorm.drive(4.5);

        claw.setPosition(0);
        api.pause(1);
        lift.setTargetPosition(SlidePresets.LOW.position);
        while (lift.isBusy()) {}

        inchWorm.drive(-35);
        inchWorm.turnDegrees(90);

        lift.setTargetPosition(SlidePresets.HIGH.position);
        while (lift.isBusy()) {}
    }
}

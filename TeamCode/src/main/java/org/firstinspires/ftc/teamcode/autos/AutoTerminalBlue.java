package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SlidePresets;

@Autonomous(name = "blue terminal - requires right side")
public class AutoTerminalBlue extends LinearOpMode {
    @Override
    public void runOpMode() {
        API api = new API(this);
        InchWorm inchWorm = new InchWorm(this);
        DcMotor lift = hardwareMap.get(DcMotor.class, "slide");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        Servo claw = hardwareMap.get(Servo.class, "claw");
        claw.scaleRange(0, 0.5);
        claw.setPosition(0);

        api.waitForStart();

        lift.setTargetPosition(SlidePresets.LOW.position);
        lift.setPower(0.75);

        // drive forward to prevent scraping against the wall
        inchWorm.drive(3.5);
        inchWorm.strafe(-13);
        inchWorm.drive(-3.5);

        lift.setTargetPosition(SlidePresets.GROUND.position);
        while (lift.isBusy()) {}
    }
}

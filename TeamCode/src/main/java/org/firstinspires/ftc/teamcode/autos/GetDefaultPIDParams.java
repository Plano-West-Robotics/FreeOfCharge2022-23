package org.firstinspires.ftc.teamcode.autos;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(group = "tune")
public class GetDefaultPIDParams extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        API api = new API(this);

        api.waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine(String.valueOf(motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION)));
            telemetry.update();
        }
    }
}

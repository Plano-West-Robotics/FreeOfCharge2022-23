package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

public class Hardware {
    public final OpMode opMode;
    public final DashboardTelemetryWrapper dashboardTelemetry;
    public final DcMotor fl;
    public final DcMotor fr;
    public final DcMotor bl;
    public final DcMotor br;
    public final DcMotor lift;
    public final Servo claw;
    public final IMU imu;
    public final List<LynxModule> hubs;

    public Hardware(OpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;
        this.opMode = opMode;
        this.dashboardTelemetry = new DashboardTelemetryWrapper(FtcDashboard.getInstance());
        opMode.telemetry = new MultipleTelemetry(opMode.telemetry, this.dashboardTelemetry);

        this.fl = hardwareMap.get(DcMotor.class, "frontRight");
        this.fr = hardwareMap.get(DcMotor.class, "frontLeft");
        this.bl = hardwareMap.get(DcMotor.class, "rearRight");
        this.br = hardwareMap.get(DcMotor.class, "rearLeft");

        this.fl.setDirection(DcMotorSimple.Direction.REVERSE);
        this.bl.setDirection(DcMotorSimple.Direction.REVERSE);
        this.fr.setDirection(DcMotorSimple.Direction.FORWARD);
        this.br.setDirection(DcMotorSimple.Direction.FORWARD);
        this.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.lift = hardwareMap.get(DcMotor.class, "spool");
        this.claw = hardwareMap.get(Servo.class, "claw");
        this.imu = hardwareMap.get(IMU.class, "imu");
        initIMU();

        this.lift.setDirection(DcMotorSimple.Direction.FORWARD);
        this.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.claw.scaleRange(0, 0.5);

        this.hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : this.hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void setClawPosition(double pos) {
        this.claw.setPosition(pos);
    }

    public void initIMU() {
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));
        imu.resetYaw();
    }

    public double getImuYaw() {
        return -this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public int getLiftEncoder() {
        return -lift.getCurrentPosition();
    }

    public void move(double powerY, double powerX, double turn, double speed) {
        double flPower = (powerY + turn + powerX) * speed;
        double frPower = (powerY - turn - powerX) * speed;
        double blPower = (powerY + turn - powerX) * speed;
        double brPower = (powerY - turn + powerX) * speed;

        double scale = Math.max(1, (Math.abs(powerY) + Math.abs(turn) + Math.abs(powerX)) * Math.abs(speed)); // shortcut for max(abs([fl,fr,bl,br]))
        flPower /= scale;
        frPower /= scale;
        blPower /= scale;
        brPower /= scale;

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }

    public void stop() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}
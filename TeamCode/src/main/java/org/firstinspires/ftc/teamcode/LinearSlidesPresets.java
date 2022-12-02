package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class LinearSlidesPresets extends LinearOpMode {

    //inches
    public static final double CLAW_TO_GROUND = 3;
    public static final double GROUND_JUNCTIONS = 0.56;
    public static final double SMALL_JUNCTIONS = 13.5;
    public static final double MEDIUM_JUNCTIONS = 23.5;
    public static final double TALL_JUNCTIONS = 33.5;
    public static final double CIRCUMFERENCE = 4.40945;
    public static final double TICKS_PER_REV = 537.7;

    //motor: 5203 Series Yellow GoBilda motor
    // RPM: 223
    // spool = 4.40945




    //Ticks per revolutions: 537.7
    // num of inches / circum  * ticks


   double targetPosGround = (GROUND_JUNCTIONS/CIRCUMFERENCE) * TICKS_PER_REV;
   double targetPosSmall =  (SMALL_JUNCTIONS/CIRCUMFERENCE) * TICKS_PER_REV;
   double targetPosMedium =  (MEDIUM_JUNCTIONS/CIRCUMFERENCE) * TICKS_PER_REV;
   double targetPosTall =  (TALL_JUNCTIONS/CIRCUMFERENCE) * TICKS_PER_REV;


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor liftMotor = hardwareMap.get(DcMotor.class, "slide");
        waitForStart();
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        while (opModeIsActive()) {
            telemetry.addData("Ticks: ", liftMotor.getCurrentPosition());
            telemetry.update();

            liftMotor.setTargetPosition((int) targetPosSmall);
            liftMotor.setPower(0.5);

            while (liftMotor.isBusy()) {

            }
        }
    }


    //switch case






}

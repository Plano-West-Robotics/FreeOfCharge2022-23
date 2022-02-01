package org.firstinspires.ftc.teamcode.createdcode.automodes;

import static org.firstinspires.ftc.teamcode.createdcode.automodes.AutoConstants.BLUE1_START_X;
import static org.firstinspires.ftc.teamcode.createdcode.automodes.AutoConstants.BLUE1_START_Y;
import static org.firstinspires.ftc.teamcode.createdcode.automodes.AutoConstants.CAROUSEL_POWER;
import static org.firstinspires.ftc.teamcode.createdcode.automodes.AutoConstants.CAROUSEL_WAIT;
import static org.firstinspires.ftc.teamcode.createdcode.automodes.AutoConstants.BLUE_CAROUSEL_X;
import static org.firstinspires.ftc.teamcode.createdcode.automodes.AutoConstants.BLUE_CAROUSEL_Y;
import static org.firstinspires.ftc.teamcode.createdcode.automodes.AutoConstants.BLUE_FINAL_STRAFE;
import static org.firstinspires.ftc.teamcode.createdcode.automodes.AutoConstants.ARM_BACK_DIST;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.roadRunner.drive.*;




@Autonomous
public class AutonomousTester extends LinearOpMode {
    private Servo grabServo;
    private CRServo carouselServo;
    private DcMotor armOne;
    private DcMotor armTwo;

    @Override
    public void runOpMode(){
        Pose2d startPos = new Pose2d(BLUE1_START_X, BLUE1_START_Y, Math.toRadians(90));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory part1 = drive.trajectoryBuilder(startPos)
                .forward(41)
                .build();

        Trajectory part2 = drive.trajectoryBuilder(part1.end().plus(new Pose2d(0,0, -1 * Math.toRadians(90))))
                .splineTo(new Vector2d(BLUE_CAROUSEL_X, BLUE_CAROUSEL_Y), Math.toRadians(0))
                .build();

        Trajectory part5 = drive.trajectoryBuilder(part2.end())
                .strafeLeft(BLUE_FINAL_STRAFE)
                .build();


        drive.setPoseEstimate(startPos);

        grabServo = hardwareMap.get(Servo.class, "grabServo");
        carouselServo = hardwareMap.get(CRServo.class, "spinnyBoy");
        armOne = hardwareMap.get(DcMotor.class, "armOne");
        armTwo = hardwareMap.get(DcMotor.class, "armTwo");

        initStuff();

        waitForStart();

        grabServo.setPosition(0);
        //telemetry.addData("grabServo", grabServo.getController());
        //telemetry.update();
        sleep(1000);
        moveArms(ARM_BACK_DIST /2);
        drive.followTrajectory(part1);
        drive.turn(-1*Math.toRadians(90));
        moveArms(ARM_BACK_DIST /4);
        grabServo.setPosition(0.4);
        sleep(500);
        moveArms(-ARM_BACK_DIST);
        drive.followTrajectory(part2);

        carouselServo.setPower(-1 * CAROUSEL_POWER);
        sleep(CAROUSEL_WAIT);
        drive.followTrajectory(part5);

    }
    private void moveArms(int encoderVal){
        armOne.setTargetPosition(armOne.getCurrentPosition() - encoderVal);
        armTwo.setTargetPosition(armTwo.getCurrentPosition() - encoderVal);
        armOne.setPower(0.5);
        armTwo.setPower(0.5);
        sleep(1000);
    }

    private void initStuff(){
        armOne.setDirection(DcMotorSimple.Direction.FORWARD);
        armTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        armOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}

package org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes;

import static org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes.AutoConstants.RED_PARK_START_X;
import static org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes.AutoConstants.RED_PARK_START_Y;
import static org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes.AutoConstants.RED_WAREHOUSE_X;
import static org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes.AutoConstants.RED_WAREHOUSE_Y;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Autonomous(group = "Red")
public class redParkOnly extends LinearOpMode {


    @Override
    public void runOpMode() {
        Pose2d startPos = new Pose2d(RED_PARK_START_X, RED_PARK_START_Y, Math.toRadians(0));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory moveIntoWarehouse = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(RED_WAREHOUSE_X, RED_PARK_START_Y, Math.toRadians(0)))
                .build();

        Trajectory moveToPark = drive.trajectoryBuilder(moveIntoWarehouse.end())
                .lineToLinearHeading(new Pose2d(RED_WAREHOUSE_X, RED_WAREHOUSE_Y, Math.toRadians(0)))
                .build();


        drive.setPoseEstimate(startPos);


        initStuff();
        waitForStart();


        sleep(500);

        drive.followTrajectory(moveIntoWarehouse);
        drive.followTrajectory(moveToPark);
    }

    private void initStuff() {


    }

}

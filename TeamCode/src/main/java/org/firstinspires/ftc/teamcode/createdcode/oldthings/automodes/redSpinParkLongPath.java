package org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes;


import static org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes.AutoConstants.RED1_BLOCK_MIDPOS_X;
import static org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes.AutoConstants.RED1_BLOCK_REPOS_X;
import static org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes.AutoConstants.RED1_MIDPOS_X;
import static org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes.AutoConstants.RED1_MIDPOS_Y;


import static org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes.AutoConstants.RED_FINAL_STRAFE;


import static org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes.AutoConstants.RED1_START_X;
import static org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes.AutoConstants.RED1_START_Y;

import static org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes.AutoConstants.RED_START_FORWARD;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.drive.*;


@Disabled
@Autonomous(group = "Red")
public class redSpinParkLongPath extends LinearOpMode {


    @Override
    public void runOpMode() {
        Pose2d startPos = new Pose2d(RED1_START_X, RED1_START_Y, Math.toRadians(90));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory moveToBlockDrop = drive.trajectoryBuilder(startPos)
                .splineToLinearHeading(new Pose2d(RED1_MIDPOS_X, RED1_MIDPOS_Y, Math.toRadians(90)), Math.toRadians(90))
                .build();

        Trajectory moveToBlockDrop2 = drive.trajectoryBuilder(moveToBlockDrop.end())
                .splineToLinearHeading(new Pose2d(RED1_START_X + RED1_BLOCK_REPOS_X, RED1_START_Y + RED_START_FORWARD, Math.toRadians(180)), Math.toRadians(0))
                .build();

        drive.setPoseEstimate(startPos);


        initStuff();
        waitForStart();


        drive.followTrajectory(moveToBlockDrop);
        drive.followTrajectory(moveToBlockDrop2);


        sleep(500);


        //drive.followTrajectory(park);
    }

    private void initStuff() {


    }
}

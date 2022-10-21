package org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes;

import static org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes.AutoConstants.BLUE1_START_X;
import static org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes.AutoConstants.BLUE1_START_Y;


import static org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes.AutoConstants.BLUE_FINAL_STRAFE;


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
@Autonomous
public class AutonomousTester extends LinearOpMode {


    @Override
    public void runOpMode() {
        Pose2d startPos = new Pose2d(BLUE1_START_X, BLUE1_START_Y, Math.toRadians(90));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory part1 = drive.trajectoryBuilder(startPos)
                .forward(41)
                .build();

        /*
        Trajectory part2 = drive.trajectoryBuilder(part1.end().plus(new Pose2d(0, 0, -1 * Math.toRadians(90))))

                .build();

        Trajectory part5 = drive.trajectoryBuilder(part2.end())
                .strafeLeft(BLUE_FINAL_STRAFE)
                .build();
         */


        drive.setPoseEstimate(startPos);


        //initStuff();

        waitForStart();


        //telemetry.update();
        sleep(1000);

        drive.followTrajectory(part1);
        drive.turn(-1 * Math.toRadians(90));


        sleep(500);

        //drive.followTrajectory(part2);


        //drive.followTrajectory(part5);

    }

    /*





        sleep(1000);
    }

    private void initStuff() {










    }
     */
}

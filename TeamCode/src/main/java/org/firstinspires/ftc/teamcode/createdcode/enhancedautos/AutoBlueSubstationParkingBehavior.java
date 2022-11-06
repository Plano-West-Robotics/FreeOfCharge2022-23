package org.firstinspires.ftc.teamcode.createdcode.enhancedautos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class AutoBlueSubstationParkingBehavior extends LinearOpMode {
    SampleMecanumDrive mecanumDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        API api = new API(this);


        waitForStart();

        // init move to default terminal
        Trajectory endpos = api.makeTrajectories(
                mecanumDrive,
                new Pose2d(new Vector2d(-34, 60), 0),
                new int[]{-34, 1},
                new int[]{60, 60}
        );

        mecanumDrive.followTrajectory(endpos);
    }
}

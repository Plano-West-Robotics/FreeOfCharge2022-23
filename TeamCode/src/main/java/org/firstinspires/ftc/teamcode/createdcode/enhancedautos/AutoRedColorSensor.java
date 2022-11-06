package org.firstinspires.ftc.teamcode.createdcode.enhancedautos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class AutoRedColorSensor extends LinearOpMode {
    SampleMecanumDrive mecanumDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensor sensor = hardwareMap.get(ColorSensor.class, "color");
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        API api = new API(this);


        waitForStart();

        Trajectory firstMove = api.makeTrajectories(
                mecanumDrive,
                new Pose2d(new Vector2d(-34, -60), 0),
                new int[]{-34},
                new int[]{-40}
        );

        mecanumDrive.followTrajectory(firstMove);

        api.pause(5);
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        int largest = api.getLargest(r, g, b);

        Trajectory endpos;
        if (largest == r) {
            // move to position 1
            endpos = api.makeTrajectories(
                    mecanumDrive,
                    firstMove.end(),
                    new int[]{-34, -12, -12},
                    new int[]{-60, -60, -12}
            );
        } else if (largest == g) {
            // move to position 2
            endpos = api.makeTrajectories(
                    mecanumDrive,
                    firstMove.end(),
                    new int[]{-34, -12, -12, -34},
                    new int[]{-60, -60, -12, -12}
            );
        } else if (largest == b) {
            // move to position 3
            endpos = api.makeTrajectories(mecanumDrive, firstMove.end(),
                    new int[]{-34, -70, -60},
                    new int[]{-60, -60, -30}
            );
        } else {
            // Color sensor broke, move to backup position
            endpos = api.makeTrajectories(mecanumDrive, firstMove.end(),
                    new int[]{-34, -70},
                    new int[]{-60, -60}
            );
        }

        mecanumDrive.followTrajectory(endpos);
    }
}

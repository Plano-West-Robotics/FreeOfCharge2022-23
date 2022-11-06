package org.firstinspires.ftc.teamcode.createdcode.enhancedautos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class API {
    OpMode opMode;

    public API(OpMode opMode) {
        this.opMode = opMode;
    }

    public void pause(double seconds) {
        double time = opMode.getRuntime() + seconds;
        while (true) {
            if (!(opMode.getRuntime() < time)) break; // stop android studio from yelling
        }
    }


    public Trajectory makeTrajectories(SampleMecanumDrive mecanumDrive, Pose2d startPos, int[] x, int[] y) {
        assert x.length == y.length;
        TrajectoryBuilder builder = mecanumDrive.trajectoryBuilder(startPos);

        for (int i = 0; i < x.length; i++) {
            builder.splineTo(new Vector2d(x[i], y[i]), 0);
        }

        return builder.build();
    }

    public int getLargest(int x, int y, int z) {
        return Math.max(z, Math.max(x, y));
    }
}

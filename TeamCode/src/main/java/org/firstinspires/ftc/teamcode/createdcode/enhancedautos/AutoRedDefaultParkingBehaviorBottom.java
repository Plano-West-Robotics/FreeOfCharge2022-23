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
public class AutoRedDefaultParkingBehaviorBottom extends LinearOpMode {
    SampleMecanumDrive mecanumDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);


        waitForStart();

        Trajectory firstMove = makeTrajectories(
                new Pose2d(new Vector2d(-34, -60), 0),
                new int[]{-34},
                new int[]{-40}
        );

        mecanumDrive.followTrajectory(firstMove);

        double time = getRuntime() + 5;
        /* wait for 5 seconds to allow color sensor to settle */
        while (true) {
            if (!(getRuntime() < time)) break; // needed to stop android studio from yelling
        }
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        int largest = getLargest(r, g, b);

        // init move to default terminal
        Trajectory endpos = makeTrajectories(firstMove.end(),
                new int[]{-34, -70},
                new int[]{60, -60}
            );
        }

        mecanumDrive.followTrajectory(endpos);
    }

    private Trajectory makeTrajectories(Pose2d startPos, int[] x, int[] y) {
        assert x.length == y.length;
        TrajectoryBuilder builder = mecanumDrive.trajectoryBuilder(startPos);

        for (int i = 0; i < x.length; i++) {
            builder.splineTo(new Vector2d(x[i], y[i]), 0);
        }

        return builder.build();
    }

    private int getLargest(int x, int y, int z) {
        return Math.max(z, (Math.max(x, y)));
    }
}

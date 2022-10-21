package org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes;

import static org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes.AutoConstants.BLUE1_START_X;
import static org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes.AutoConstants.BLUE1_START_Y;
import static org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes.AutoConstants.BLUE_START_FORWARD;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Disabled
@Autonomous(group = "Blue")
public class blueSpinPark extends LinearOpMode {
    /*




     */

    @Override
    public void runOpMode() {
        Pose2d startPos = new Pose2d(BLUE1_START_X, BLUE1_START_Y, -1 * Math.toRadians(90));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory moveToBlockDrop = drive.trajectoryBuilder(startPos)
                .splineToLinearHeading(new Pose2d(BLUE1_START_X - 2, BLUE1_START_Y - BLUE_START_FORWARD, Math.toRadians(180)), Math.toRadians(0))
                .build();

        /*


                .build();
         */

        /*


                .build();
         */


        drive.setPoseEstimate(startPos);

        /*




         */

        //initStuff();

        waitForStart();

        /*




        drive.followTrajectory(moveToBlockDrop);


        sleep(500);





        drive.followTrajectory(park);
         */

    }

    /*





        sleep(1000);
    }

    private void initStuff() {










    }
     */
}

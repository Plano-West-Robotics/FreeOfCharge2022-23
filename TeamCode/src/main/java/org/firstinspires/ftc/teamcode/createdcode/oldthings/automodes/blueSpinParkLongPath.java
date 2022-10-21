package org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes;


import static org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes.AutoConstants.BLUE1_START_X;
import static org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes.AutoConstants.BLUE1_START_Y;
import static org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes.AutoConstants.BLUE_START_FORWARD;


import static org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes.AutoConstants.BLUE1_BLOCK_MIDPOS_X;
import static org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes.AutoConstants.BLUE1_BLOCK_REPOS_X;
import static org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes.AutoConstants.BLUE1_MIDPOS_X;
import static org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes.AutoConstants.BLUE1_MIDPOS_Y;


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

@Autonomous(group = "Blue")
public class blueSpinParkLongPath extends LinearOpMode {


    @Override
    public void runOpMode() {
        Pose2d startPos = new Pose2d(BLUE1_START_X, BLUE1_START_Y, -1 * Math.toRadians(90));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory moveToBlockDrop = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(BLUE1_MIDPOS_X, BLUE1_MIDPOS_Y, -1 * Math.toRadians(90)))
                .build();

        Trajectory moveToBlockDrop2 = drive.trajectoryBuilder(moveToBlockDrop.end())
                .lineToLinearHeading(new Pose2d(BLUE1_START_X + BLUE1_BLOCK_REPOS_X, BLUE1_START_Y - BLUE_START_FORWARD, -1 * Math.toRadians(180)))
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

/*
package org.firstinspires.ftc.teamcode.createdcode.driveobjs;










import static org.firstinspires.ftc.teamcode.createdcode.configs.AutoConfig.SLIGHTLY_OFF_GROUND;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class EnhancedDriver extends SampleMecanumDrive {

    private final HardwareMap hardwareMap;




    private Pose2d lastPose = null;
    private boolean isFirstRun = true;
    private final TelemetryPacket packet = new TelemetryPacket();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();


        OPEN,
        CLOSED_EMPTY,
        CLOSED_FULL
    }

    public EnhancedDriver(HardwareMap hardwareMap) {
        super(hardwareMap);
        this.hardwareMap = hardwareMap;
        init();
    }

    public void act(ActionObject args) {
        if (isFirstRun) {
            lastPose = args.getPose2d();
            setPoseEstimate(lastPose);
            isFirstRun = false;
        } else {
            Pose2d newPose = args.getPose2d();
            //checks to make sure that the poses are different, avoids no trajectory error
            if (!(newPose.getX() == lastPose.getX() && newPose.getY() == lastPose.getY())) {
                Trajectory traj = trajectoryBuilder(lastPose)
                        .lineToLinearHeading(newPose)
                        .build();
                new Pose2d();
                followTrajectory(traj);
                lastPose = newPose;
            }
        }
        if (args.getMethodID() != 0) {
            try {
                doAction(args.getMethodID());
            } catch (IndexOutOfBoundsException e) {
                packet.put("Invalid Method ID", args.getMethodID() + " is not a valid ID");
                flushTelemetry();
            }
        }
    }

    public void doAction(int id) throws IndexOutOfBoundsException {
        int subIndex = id % 10;
        switch (id / 10) {
            case 1:

                break;
            case 2:

                break;
            case 3:

                break;
            case 4:
                executeAdvancedMovement(subIndex);
        }

    }


        switch (id) {

            case 0:

                break;

            case 1:

                break;

            case 2:

                break;

            case 3:

                break;

            case 4:

                break;
            /* currently unnecessary

            case 5:

                break;

            case 6:

                break;

            case 7:

            */
//throws error
/*
            default:

        }
    }


        switch (id) {

            case 0:



                break;

            case 1:


                break;

            case 2:

                break;
            //throws error
            default:

        }

    }


        switch (id) {
            //turn right
            case 1:



                break;
            //turn left
            case 2:



                break;
        }



    }

    private void executeAdvancedMovement(int id) throws IndexOutOfBoundsException {
//        switch(id){
//            //moves to the leftmost edge (blue side) and drives into warehouse
//            case 1:
//                Trajectory traj = trajectoryBuilder(getPoseEstimate())
//                    .lineToLinearHeading()
//                    .lineToLinearHeading();
//
//
//            //moves to the rightmost edge (red side) and drives into warehouse
//            case 2:
//
//
//        }
    }









    }

    private void sleep(long milli) {
        try {
            Thread.sleep(milli);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }


    private void flushTelemetry() {
        dashboard.sendTelemetryPacket(packet);
    }

    public void init() {












    }


}
    */
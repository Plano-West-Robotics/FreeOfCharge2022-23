package org.firstinspires.ftc.teamcode.createdcode.driveobjs;





import static org.firstinspires.ftc.teamcode.createdcode.configs.AutoConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;

public class EnhancedDriver extends SampleMecanumDrive{

    HardwareMap hardwareMap;
    private Servo grabServo;
    private CRServo carouselServo;
    private DcMotor armOne;
    private DcMotor armTwo;
    private Pose2d lastPose;
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public enum GrabState {
        OPEN,
        CLOSED_EMPTY,
        CLOSED_FULL
    };

    public EnhancedDriver(HardwareMap hardwareMap){
        super(hardwareMap);
        this.hardwareMap = hardwareMap;
        init();

    }
     public void act(ActionObject args) {
         Pose2d newPose = args.getPose2d();

         //checks to make sure that the poses are different, avoids no trajectory error
         if (!(newPose.getX() == lastPose.getX() && newPose.getY() == lastPose.getY())) {
             Trajectory traj = trajectoryBuilder(lastPose)
                     .lineToLinearHeading(newPose)
                     .build();
             new Pose2d();
             followTrajectory(traj);
         }

         try {
            doAction(args.getMethodID());
         }
         catch (IndexOutOfBoundsException e){
            packet.put("Invalid Method ID", args.getMethodID() + " is not a valid ID");
            flushTelemetry();
         }
    }

    public void doAction(int id) throws IndexOutOfBoundsException{
        int subIndex = id%10;
        switch(id/10){
            case 1:
                moveArmMethods(subIndex);
                break;
            case 2:
                moveGrabberMethods(subIndex);
                break;
            default:
                throw new IndexOutOfBoundsException("Invalid Method Type");
        }

    }

    private void moveArmMethods(int id) throws IndexOutOfBoundsException {
        switch(id){
            //lifts arm to quarter
            case 1:
                moveArms(ARM_MAX_DIST/4);
                break;
            //lifts arm to half
            case 2:
                moveArms(ARM_MAX_DIST/2);
                break;
            //lifts arm to three-quarters
            case 3:
                moveArms((ARM_MAX_DIST*3)/4);
                break;
            //lifts arm to full
            case 4:
                moveArms(ARM_MAX_DIST);
                break;
            //throws error
            default:
                throw new IndexOutOfBoundsException("Not a valid arm method");
        }
    }

    private void moveGrabberMethods(int id) throws IndexOutOfBoundsException{
        switch(id){
            //close grabber
            case 1:
                grabServo.setPosition(GRABBER_CLOSE);
                break;
            //open grabber
            case 2:
                grabServo.setPosition(GRABBER_OPEN);
                break;
            //throws error
            default:
                throw new IndexOutOfBoundsException("Not a valid grabber method");
        }
    }


    private void moveArms(int encoderVal){
        armOne.setTargetPosition(armOne.getCurrentPosition() - encoderVal);
        armTwo.setTargetPosition(armTwo.getCurrentPosition() - encoderVal);
        armOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armOne.setPower(0.5);
        armTwo.setPower(0.5);
    }


    public void flushTelemetry(){
        dashboard.sendTelemetryPacket(packet);
    }

    public void init(){
        grabServo = hardwareMap.get(Servo.class, "grabServo");
        carouselServo = hardwareMap.get(CRServo.class, "spinnyBoy");
        armOne = hardwareMap.get(DcMotor.class, "armOne");
        armTwo = hardwareMap.get(DcMotor.class, "armTwo");
    }

}

package org.firstinspires.ftc.teamcode.createdcode.driveobjs;





import static org.firstinspires.ftc.teamcode.createdcode.configs.AutoConfig.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class EnhancedDriver extends SampleMecanumDrive{

    private HardwareMap hardwareMap;
    private Servo grabServo;
    private DcMotor carouselMotor1, carouselMotor2;
    private DcMotor armOne;
    private DcMotor armTwo;
    private Pose2d lastPose = null;
    private boolean isFirstRun = true;
    private TelemetryPacket packet = new TelemetryPacket();
    private FtcDashboard dashboard = FtcDashboard.getInstance();

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
         if (isFirstRun){
             lastPose = args.getPose2d();
             setPoseEstimate(lastPose);
             isFirstRun = false;
         }
         else {
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

    public void doAction(int id) throws IndexOutOfBoundsException{
        int subIndex = id%10;
        switch(id/10){
            case 1:
                moveArmMethods(subIndex);
                break;
            case 2:
                moveGrabberMethods(subIndex);
                break;
            case 3:
                turnCarouselMethods(subIndex);
                break;

        }

    }

    private void moveArmMethods(int id) throws IndexOutOfBoundsException {
        switch(id){
            //resets arm position
            case 0:
                moveArms(armOne.getCurrentPosition());
                break;
            //lifts arm to top layer from front
            case 1:
                moveArms(LAYER_ONE_FRONT_ARM_POS);
                break;
            //lifts arm to middle layer from front
            case 2:
                moveArms(LAYER_TWO_FRONT_ARM_POS);
                break;
            //lifts arm to bottom layer from front
            case 3:
                moveArms(LAYER_THREE_FRONT_ARM_POS);
                break;
            //lifts arm to full
            case 4:
                moveArms(ARM_MAX_DIST);
                break;
            /* currently unnecessary
            //lifts arm to front layer from back
            case 5:
                moveArms(LAYER_ONE_BACK_ARM_POS);
                break;
            //lifts arm to middle layer from back
            case 6:
                moveArms(LAYER_TWO_BACK_ARM_POS);
                break;
            //lifts arm to bottom layer from back
            case 7:
                moveArms(LAYER_THREE_BACK_ARM_POS);
            */
            //throws error
            default:
                throw new IndexOutOfBoundsException("Not a valid arm method");
        }
    }

    private void moveGrabberMethods(int id) throws IndexOutOfBoundsException{
        switch(id){
            //close grabber and raises a little off ground
            case 0:
                grabServo.setPosition(GRABBER_CLOSE);
                sleep(GRAB_TIME);
                moveArms(SLIGHTLY_OFF_GROUND);
                break;
            //close grabber
            case 1:
                grabServo.setPosition(GRABBER_CLOSE);
                sleep(GRAB_TIME);
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

    private void turnCarouselMethods(int id) throws IndexOutOfBoundsException{
        switch(id) {
            //turn right
            case 1:
                carouselMotor1.setPower(CAROUSEL_POWER);
                carouselMotor2.setPower(CAROUSEL_POWER);

                break;
            //turn left
            case 2:
                carouselMotor1.setPower(-1*CAROUSEL_POWER);
                carouselMotor2.setPower(-1*CAROUSEL_POWER);

                break;
        }
        sleep(CAROUSEL_WAIT);
        carouselMotor1.setPower(0);
        carouselMotor2.setPower(0);
    }


    private void moveArms(int encoderVal){
        armOne.setTargetPosition(armOne.getCurrentPosition() - encoderVal);
        armTwo.setTargetPosition(armTwo.getCurrentPosition() - encoderVal);
        armOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armOne.setPower(0.5);
        armTwo.setPower(0.5);
    }

    private void sleep(long milli){
        try{
            Thread.sleep(milli);
        }
        catch (InterruptedException e){
            e.printStackTrace();
        }
    }




    private void flushTelemetry(){
        dashboard.sendTelemetryPacket(packet);
    }

    public void init(){
        grabServo = hardwareMap.get(Servo.class, "grabServo");
        carouselMotor1 = hardwareMap.get(DcMotor.class, "spinnyBoyOne");
        carouselMotor2 = hardwareMap.get(DcMotor.class, "spinnyBoyTwo");
        armOne = hardwareMap.get(DcMotor.class, "armOne");
        armTwo = hardwareMap.get(DcMotor.class, "armTwo");
        armOne.setDirection(DcMotorSimple.Direction.REVERSE);
        armOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }



}

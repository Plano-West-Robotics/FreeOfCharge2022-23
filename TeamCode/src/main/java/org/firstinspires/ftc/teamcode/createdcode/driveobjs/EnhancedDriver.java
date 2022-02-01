package org.firstinspires.ftc.teamcode.createdcode.driveobjs;





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

    public EnhancedDriver(HardwareMap hardwareMap){
        super(hardwareMap);
        this.hardwareMap = hardwareMap;

        grabServo = hardwareMap.get(Servo.class, "grabServo");
        carouselServo = hardwareMap.get(CRServo.class, "spinnyBoy");
        armOne = hardwareMap.get(DcMotor.class, "armOne");
        armTwo = hardwareMap.get(DcMotor.class, "armTwo");
    }
     public void runLocations();

}

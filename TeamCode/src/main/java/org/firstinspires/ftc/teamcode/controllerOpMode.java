package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class controllerOpMode extends OpMode {
    private DcMotor frontRight, frontLeft, rearRight, rearLeft, spool, spinnyBoy, intakeMotor;
    private double drive,strafe, turn, linearSpeed;
    private double speed;
    MotorBox driveMotors;



    // part that actually runs
    // naturally loops
    public void loop() {

        // checks to see if the speed change button is pressed
        detectSpeedChange();
        detectSlideSpeedChange();

        // sets the driver strafe and turn values
        // drive is negated so 1 is forward
        drive = -1*gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;

        // prints the speed
        telemetry.addData("Speed", "Current Speed = " + Math.round(speed*100));
        telemetry.addData("Linear Slide SPeed", "Current Speed = " + Math.round(linearSpeed*100));

        // makes it drive
        // result is just debug values
        String result = driveMotors.drivePower(drive, strafe, turn, speed);
        spool.setPower(gamepad2.left_stick_y*linearSpeed);
        if(gamepad2.b){
            spinnyBoy.setPower(0.5);
        }
        else {
            spinnyBoy.setPower(0);
        }
        if(gamepad2.a){
            intakeMotor.setPower(0.5);
        }
        else {
            intakeMotor.setPower(0);
        }


        //debugging
        //telemetry.addData("Results", result);
        /*
        telemetry.addData("Drive", "Drive = " + drive);
        telemetry.addData("Strafe", "Strafe = " + strafe);
        */

        //show telemetry
        telemetry.update();
    }

    // initializes the driveMotors MotorBox and speed
    public void init(){
        driveMotors = new MotorBox(
                hardwareMap.get(DcMotor.class, "frontRight"),
                hardwareMap.get(DcMotor.class, "frontLeft"),
                hardwareMap.get(DcMotor.class, "rearRight"),
                hardwareMap.get(DcMotor.class, "rearLeft"),
                false
        );
        speed = 0.25;
        linearSpeed = 0.25;
        spool = hardwareMap.get(DcMotor.class, "spool");
        spinnyBoy = hardwareMap.get(DcMotor.class, "spinnyBoy");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        spool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // detects the speed change button and changes speed accordingly
    public void detectSpeedChange(){
        if (gamepad1.dpad_up){
            if (speed <= 1) {
                speed += 0.0005;
            }
        }
        if (gamepad1.dpad_down){
            if (speed >= 0) {
                speed -= 0.0005;
            }
        }
    }
    public void detectSlideSpeedChange(){
        if (gamepad2.dpad_up){
            if (linearSpeed <= 1) {
                linearSpeed += 0.0005;
            }
        }
        if (gamepad2.dpad_down){
            if (linearSpeed >= 0) {
                linearSpeed  -= 0.0005;
            }
        }
    }


}

package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.kinematics.FieldVector;
import org.firstinspires.ftc.teamcode.kinematics.NullablePose;
import org.firstinspires.ftc.teamcode.kinematics.Poser;
import org.firstinspires.ftc.teamcode.kinematics.LiftPosition;

@Autonomous(group = "test")
public class PoserTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Hardware hardware = new Hardware(this);
        Poser poser = new Poser(hardware, 0.5, FieldVector.ZERO, 0);

        poser.goTo(new NullablePose());

        waitForStart();

        while (opModeIsActive()) {
            poser.goTo(new NullablePose(FieldVector.inTiles(2, 0), 90, LiftPosition.GROUND));
            sleep(1000);
            poser.goTo(new NullablePose(FieldVector.inTiles(0, 1), -135, LiftPosition.HIGH));
            sleep(1000);
            poser.goTo(new NullablePose(FieldVector.ZERO, 0, LiftPosition.MEDIUM));
            sleep(1000);
        }
    }
}

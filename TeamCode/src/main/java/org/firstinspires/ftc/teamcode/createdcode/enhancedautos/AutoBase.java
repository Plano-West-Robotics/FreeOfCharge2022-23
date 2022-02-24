package org.firstinspires.ftc.teamcode.createdcode.enhancedautos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
@Config
public class AutoBase extends enhancedAutoMode{
    public static double[] xCoordArr = {};
    private static double[] yCoordArr = {};
    private static double[] angleArr = {};
    private static int[] methodIdArr = {};

    @Override
    public void runOpMode(){
        initThings();

        setXCoords(xCoordArr);
        setYCoords(yCoordArr);
        setAngles(angleArr);
        setMethodIDS(methodIdArr);

        try {
            makeActionObjects();
        } catch (Exception e) {
            e.printStackTrace();
        }

        waitForStart();

        run();
    }

}

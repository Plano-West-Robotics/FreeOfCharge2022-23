package org.firstinspires.ftc.teamcode.createdcode.enhancedautos;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class redAuto extends enhancedAutoMode{
    private static double[] xCoordArr = {0, 1, 2};
    private static double[] yCoordArr = {0, 1, 2};
    private static double[] angleArr = {90, 90, 90};
    private static int[] methodIdArr = {11, 12, 11};

    @Override
    public void runOpMode(){
        initThings();

        setXCoords(xCoordArr);
        setYCoords(yCoordArr);
        setAngles(angleArr);
        setMethodIDS(methodIdArr);

        waitForStart();

        run();
    }

}

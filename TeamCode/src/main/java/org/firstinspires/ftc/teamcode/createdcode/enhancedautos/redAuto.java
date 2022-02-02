package org.firstinspires.ftc.teamcode.createdcode.enhancedautos;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class redAuto extends enhancedAutoMode{
    private static double[] xCoordArr = {0, 1, 2};
    private static double[] yCoordArr = {0, 1, 2};
    private static double[] angleArr = {0, 1, 2};
    private static int[] methodIdArr = {0, 1};

    @Override
    public void runOpMode(){
        initThings();



        xCoords = Arrays.asList(xCoordArr);
        yCoords = Arrays.asList(yCoordArr);
        angles = Arrays.asList(angleArr);
        methodIDs = Arrays.asList();
        int[] arrayTest = {0, 0};


        waitForStart();

        run();
    }

}

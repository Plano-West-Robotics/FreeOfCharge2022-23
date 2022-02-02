package org.firstinspires.ftc.teamcode.createdcode.oldthings.automodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.createdcode.driveobjs.ActionObject;
import org.firstinspires.ftc.teamcode.createdcode.driveobjs.EnhancedDriver;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public abstract class enhancedAutoMode extends LinearOpMode {
    private static List<Double> xCoords = new ArrayList<>(Arrays.asList());
    private static List<Double> yCoords = new ArrayList<>(Arrays.asList());
    private static List<Double> angles = new ArrayList<>(Arrays.asList());
    private static List<Integer> methodIDs = new ArrayList<>(Arrays.asList());
    Exception invalidValues = new Exception("List of values is invalid");
    private static List<ActionObject> actionObjects;
    EnhancedDriver enhancedDriver;

    public void makeActionObjects() throws Exception{
        if (xCoords.get(0) == null || yCoords.get(0) == null || angles.get(0) == null)
            throw invalidValues;
        if (!(xCoords.size() == yCoords.size() && xCoords.size() == angles.size() && xCoords.size() == methodIDs.size()))
            throw invalidValues;


        for (int i = 0; i < xCoords.size(); i++){
            double x = checkNull(xCoords, i);
            double y = checkNull(yCoords, i);
            double angle = checkNull(angles, i);

            ActionObject newAction  = new ActionObject(x, y, angle, methodIDs.get(i));
            actionObjects.add(newAction);
        }
    }
    public void run(){
        enhancedDriver = new EnhancedDriver(hardwareMap);

        waitForStart();

        for (ActionObject i : actionObjects){
            enhancedDriver.act(i);
        }
    }

    public double checkNull(List<Double> toCheck, int i){
        return  toCheck.get(i) == null ? checkNull(toCheck, i) : toCheck.get(i);
    }

}

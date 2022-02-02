package org.firstinspires.ftc.teamcode.createdcode.enhancedautos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.createdcode.driveobjs.ActionObject;
import org.firstinspires.ftc.teamcode.createdcode.driveobjs.EnhancedDriver;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public abstract class enhancedAutoMode extends LinearOpMode {
    private static List<Double> xCoords = new ArrayList<>(0);
    private static List<Double> yCoords = new ArrayList<>(0);
    private static List<Double> angles = new ArrayList<>(0);
    private static List<Integer> methodIDs = new ArrayList<>(0);
    private static Exception invalidValues = new Exception("List of values is invalid");
    private static List<ActionObject> actionObjects;
    private EnhancedDriver enhancedDriver;

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

        for (ActionObject i : actionObjects){
            enhancedDriver.act(i);
        }
    }

    public double checkNull(List<Double> toCheck, int i){
        return  toCheck.get(i) == null ? checkNull(toCheck, i) : toCheck.get(i);
    }

    public void initThings(){
        enhancedDriver = new EnhancedDriver(hardwareMap);
    }

    public void setXCoords(double[] xArr){
        for (Double i : xArr)
            xCoords.add(i);
    }
    public void setYCoords(double[] yArr){
        for (Double i : yArr)
            yCoords.add(i);
    }
    public void setAngles(double[] angleArr){
        for (Double i : angleArr)
            angles.add(i);
    }
    public void setMethodIDS(int[] methodIdsArr){
        for (int i : methodIdsArr)
            methodIDs.add(i);
    }

}

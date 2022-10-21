package org.firstinspires.ftc.teamcode.createdcode.enhancedautos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.createdcode.driveobjs.ActionObject;
import org.firstinspires.ftc.teamcode.createdcode.driveobjs.ObjectDetector;

import java.util.ArrayList;
import java.util.List;

public abstract class EnhancedAutoMode extends LinearOpMode {
    private static List<Double> xCoords = new ArrayList<>(0);
    private static List<Double> yCoords = new ArrayList<>(0);
    private static List<Double> angles = new ArrayList<>(0);
    private static List<Integer> methodIDs = new ArrayList<>(0);
    private static final Exception invalidValues = new Exception("List of values is invalid");
    private static List<ActionObject> actionObjects = new ArrayList<>(0);
    //private EnhancedDriver enhancedDriver;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = dashboard.getTelemetry();


    public void makeActionObjects() throws Exception {
        actionObjects = new ArrayList<>(0);
        if (xCoords.get(0) == null || yCoords.get(0) == null || angles.get(0) == null)
            throw invalidValues;
        if (!(xCoords.size() == yCoords.size() && xCoords.size() == angles.size() && xCoords.size() == methodIDs.size()))
            throw invalidValues;


        for (int i = 0; i < xCoords.size(); i++) {
            double x = checkNull(xCoords, i);
            double y = checkNull(yCoords, i);
            double angle = checkNull(angles, i);

            ActionObject newAction = new ActionObject(x, y, angle, methodIDs.get(i));
            actionObjects.add(newAction);
            telemetry.addData("Amount of ActionObjects", actionObjects.size());
            telemetry.update();
        }
    }

    public void run() {
        int count = 0;
        for (ActionObject i : actionObjects) {
            //enhancedDriver.act(i);
            count++;
            telemetry.addData("Completed Action:", count);
            telemetry.update();
        }
    }

    public double checkNull(List<Double> toCheck, int i) {
        return toCheck.get(i) == null ? checkNull(toCheck, i) : toCheck.get(i);
    }
    public void initThings() {
        //enhancedDriver = new EnhancedDriver(hardwareMap);
    }

    public void setXCoords(double[] xArr) {
        xCoords = new ArrayList<>(0);
        for (Double i : xArr)
            xCoords.add(i);
    }

    public void setYCoords(double[] yArr) {
        yCoords = new ArrayList<>(0);
        for (Double i : yArr)
            yCoords.add(i);
    }

    public void setAngles(double[] angleArr) {
        angles = new ArrayList<>(0);
        for (Double i : angleArr)
            angles.add(i);
    }

    public void setMethodIDS(int[] methodIdsArr) {
        methodIDs = new ArrayList<>(0);
        for (int i : methodIdsArr)
            methodIDs.add(i);
    }

    public int getShippingElementPos(int h, int pos1, int pos2, int pos3) {
        ObjectDetector detector = new ObjectDetector(hardwareMap, h, pos1, pos2, pos3);
        int pos = detector.getPos();
        detector.endStream();
        return pos;
    }

}

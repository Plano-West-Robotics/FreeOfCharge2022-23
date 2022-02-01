package org.firstinspires.ftc.teamcode.createdcode.driveobjs;

public class ActionObject {
    private double x, y;
    private double angle;
    private int methodToCall;

    public ActionObject(double x, double y, double angle){
        this.x = x;
        this.y = y;
        this.angle = angle;
    }
    public ActionObject(int x, int y, double angle, int methodToCall){
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.methodToCall = methodToCall;
    }
}

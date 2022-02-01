package org.firstinspires.ftc.teamcode.createdcode.driveobjs;

import com.acmerobotics.roadrunner.geometry.Pose2d;

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

    public Pose2d getPose2d(){
        return new Pose2d(x, y, Math.toRadians(angle));
    }

}

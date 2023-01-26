package org.firstinspires.ftc.teamcode.kinematics;

public class Pose {
    public final FieldVector pos;
    public final double yaw;
    public final int liftPos;

    public final static Pose ZERO = new Pose(
            FieldVector.ZERO,
            0,
            0
    );

    public Pose(FieldVector pos, double yaw, int liftPos) {
        this.pos = pos;
        this.yaw = yaw;
        this.liftPos = liftPos;
    }
}

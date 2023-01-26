package org.firstinspires.ftc.teamcode.kinematics;

public enum LiftPosition {
    GROUND(0), LOW(1846), MEDIUM(2570), HIGH(4200);

    public final int position;

    LiftPosition(int position) {
        this.position = position;
    }
}

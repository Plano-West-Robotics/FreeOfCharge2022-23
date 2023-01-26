package org.firstinspires.ftc.teamcode.kinematics;

import androidx.annotation.Nullable;

public class NullablePose {
    @Nullable
    public final FieldVector pos;
    @Nullable
    public final Double yaw;
    @Nullable
    public final Integer liftPos;

    public NullablePose(FieldVector pos, double yaw, int liftPos) {
        this.pos = pos;
        this.yaw = yaw;
        this.liftPos = liftPos;
    }

    public NullablePose(FieldVector pos, double yaw, LiftPosition liftPos) {
        this.pos = pos;
        this.yaw = yaw;
        this.liftPos = liftPos.getEncoderVal();
    }

    public NullablePose(FieldVector pos, double yaw) {
        this.pos = pos;
        this.yaw = yaw;
        this.liftPos = null;
    }

    public NullablePose(double yaw, int liftPos) {
        this.pos = null;
        this.yaw = yaw;
        this.liftPos = liftPos;
    }

    public NullablePose(double yaw, LiftPosition liftPos) {
        this.pos = null;
        this.yaw = yaw;
        this.liftPos = liftPos.getEncoderVal();
    }

    public NullablePose(FieldVector pos, int liftPos) {
        this.pos = pos;
        this.yaw = null;
        this.liftPos = liftPos;
    }

    public NullablePose(FieldVector pos, LiftPosition liftPos) {
        this.pos = pos;
        this.yaw = null;
        this.liftPos = liftPos.getEncoderVal();
    }

    public NullablePose(FieldVector pos) {
        this.pos = pos;
        this.yaw = null;
        this.liftPos = null;
    }

    public NullablePose(double yaw) {
        this.pos = null;
        this.yaw = yaw;
        this.liftPos = null;
    }

    public NullablePose(int liftPos) {
        this.pos = null;
        this.yaw = null;
        this.liftPos = liftPos;
    }

    public NullablePose(LiftPosition liftPos) {
        this.pos = null;
        this.yaw = null;
        this.liftPos = liftPos.getEncoderVal();
    }

    public NullablePose() {
        this.pos = null;
        this.yaw = null;
        this.liftPos = null;
    }

    public NullablePose or(NullablePose other) {
        return new NullablePose(
                this.pos != null ? this.pos : other.pos,
                this.yaw != null ? this.yaw : other.yaw,
                this.liftPos != null ? this.liftPos : other.liftPos
        );
    }

    public NullablePose and(NullablePose other) {
        return other.or(this);
    }
}

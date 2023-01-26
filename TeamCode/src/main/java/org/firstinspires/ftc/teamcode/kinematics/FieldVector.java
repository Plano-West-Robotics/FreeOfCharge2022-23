package org.firstinspires.ftc.teamcode.kinematics;

public class FieldVector {
    public final Distance x;
    public final Distance y;

    public static final FieldVector ZERO = new FieldVector(Distance.ZERO, Distance.ZERO);
//    public static final FieldVector RED_LEFT_SIDE_START = FieldVector.inTiles(-1.5, -2.5);

    public static FieldVector inTiles(double x, double y) {
        return new FieldVector(Distance.inTiles(x), Distance.inTiles(y));
    }

    public static FieldVector inFeet(double x, double y) {
        return new FieldVector(Distance.inFeet(x), Distance.inFeet(y));
    }

    public static FieldVector inMM(double x, double y) {
        return new FieldVector(Distance.inMM(x), Distance.inMM(y));
    }

    public FieldVector(Distance x, Distance y) {
        this.x = x;
        this.y = y;
    }

    public String toString() {
        return "(" + this.x + ", " + this.y + ")";
    }

    public FieldVector add(FieldVector rhs) {
        return new FieldVector(this.x.add(rhs.x), this.y.add(rhs.y));
    }

    public FieldVector sub(FieldVector rhs) {
        return new FieldVector(this.x.sub(rhs.x), this.y.sub(rhs.y));
    }

    public FieldVector mul(double rhs) {
        return new FieldVector(this.x.mul(rhs), this.y.mul(rhs));
    }

    public FieldVector div(double rhs) {
        return new FieldVector(this.x.div(rhs), this.y.div(rhs));
    }

    public FieldVector rot(double rhs) {
        double sin = Math.sin(Math.toRadians(rhs));
        double cos = Math.cos(Math.toRadians(rhs));
        return new FieldVector(this.y.mul(sin).add(this.x.mul(cos)), this.y.mul(cos).sub(this.x.mul(sin)));
    }

    public FieldVector neg() {
        return new FieldVector(this.x.neg(), this.y.neg());
    }

    public Distance magnitude() {
        return Distance.inDefaultUnits(Math.sqrt(this.sqMagnitude().valInDefaultUnits()));
    }

    public Distance sqMagnitude() {
        return Distance.inDefaultUnits(
                this.x.valInDefaultUnits() * this.x.valInDefaultUnits()
                        + this.y.valInDefaultUnits() * this.y.valInDefaultUnits()
        );
    }

    public FieldVector normalized() {
        if (this.magnitude().isZero()) {
            return FieldVector.ZERO;
        } else {
            return this.div(this.magnitude().valInDefaultUnits());
        }
    }

    public double angle() {
        return 90 - Math.toDegrees(Math.atan2(this.y.valInDefaultUnits(), this.x.valInDefaultUnits()));
    }
}

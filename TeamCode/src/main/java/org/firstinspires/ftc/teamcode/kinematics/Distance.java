package org.firstinspires.ftc.teamcode.kinematics;

public class Distance {
    private final double val; // in millimeters

    public static final double IN_PER_TILE = 23.625;
    public static final double MM_PER_IN = 25.4;
    public static final double MM_PER_FOOT = 304.8; // 25.4 * 12 but without floating point problems

    public static final Distance ZERO = new Distance(0);
    public static final Distance ROBOT_LENGTH = Distance.inInches(18);
    public static final Distance ROBOT_WIDTH = Distance.inInches(15.5);

    public static final Distance TILE_BORDER = Distance.inInches(0.75);
    public static final Distance ONE_TILE_WITHOUT_BORDER = Distance.inTiles(1).sub(Distance.TILE_BORDER);
    public static final Distance ONE_TILE_WITH_BORDER = Distance.inTiles(1).add(Distance.TILE_BORDER);

    public static Distance inDefaultUnits(double val) {
        return new Distance(val);
    }

    public static Distance inTiles(double val) {
        return Distance.inInches(val * IN_PER_TILE);
    }

    public static Distance inFeet(double val) {
        return Distance.inMM(val * MM_PER_FOOT);
    }

    public static Distance inInches(double val) {
        return Distance.inMM(val * MM_PER_IN);
    }

    public static Distance inMM(double val) {
        return new Distance(val);
    }

    public double valInDefaultUnits() {
        return this.val;
    }

    public double valInTiles() {
        return this.valInInches() / IN_PER_TILE;
    }

    public double valInFeet() {
        return this.valInMM() / MM_PER_FOOT;
    }

    public double valInInches() {
        return this.valInMM() / MM_PER_IN;
    }

    public double valInMM() {
        return this.val;
    }

    private Distance(double val) {
        this.val = val;
    }

    public String toString() {
        return Double.toString(this.val);
    }

    public boolean isZero() {
        return this.val == 0;
    }

    public Distance add(Distance rhs) {
        return new Distance(this.val + rhs.val);
    }

    public Distance sub(Distance rhs) {
        return new Distance(this.val - rhs.val);
    }

    public Distance mul(double rhs) {
        return new Distance(this.val * rhs);
    }

    public Distance div(double rhs) {
        return new Distance(this.val / rhs);
    }

    public Distance neg() {
        return new Distance(-this.val);
    }
}

package org.firstinspires.ftc.teamcode.autos;

// for lack of a better name, tbh
public class InchMover {
    // TODO: tune these values
    private static final double STRAFE_IPS = 1;
    private static final double DRIVE_IPS = 1;
    private static final double SPEED = 0.5;

    private final MovementAPI movementAPI;


    public InchMover(MovementAPI movementAPI) {
        this.movementAPI = movementAPI;
    }

    /**
     * Strafes for the specified number of inches.
     * This number can be negative: positive inches indicate left, negative inches indicate right
     * @param inches number of inches to move; positive = left, negative = right
     */
    public void strafe(double inches) {
        double seconds = inches / STRAFE_IPS;
        double sign = Math.signum(seconds);
        movementAPI.moveFor(90 * sign, SPEED, Math.abs(seconds));
    }

    /**
     * Drives forward/backward for the specified number of inches.
     * This number can be negative: positive indicates forward, negative indicates backward.
     * @param inches number of inches to move; positive = forward, negative = backward
     */
    public void drive(double inches) {
        double seconds = inches / DRIVE_IPS;
        // if seconds are negative, move backwards
        double direction = Math.signum(seconds) == -1 ? 180 : 0;
        movementAPI.moveFor(direction, SPEED, Math.abs(seconds));
    }
}

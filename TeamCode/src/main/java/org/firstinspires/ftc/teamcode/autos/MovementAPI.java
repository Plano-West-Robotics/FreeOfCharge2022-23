package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MovementAPI {
    private final API api;

    private final DcMotor fl;
    private final DcMotor fr;
    private final DcMotor bl;
    private final DcMotor br;

    private boolean flipped = false;

    public DcMotor getFL() { return fl; }
    public DcMotor getFR() { return fr; }
    public DcMotor getBL() { return bl; }
    public DcMotor getBR() { return br; }

    /**
     * Initializes the MovementAPI
     */
    public MovementAPI(API api) {
        this.api = api;

        api.print("Initializing MovementApi...");

        fl = api.opMode.hardwareMap.get(DcMotor.class, "frontLeft");
        fr = api.opMode.hardwareMap.get(DcMotor.class, "frontRight");
        bl = api.opMode.hardwareMap.get(DcMotor.class, "rearLeft");
        br = api.opMode.hardwareMap.get(DcMotor.class, "rearRight");

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
    }

    public void setFlipped(boolean flipped) {
        this.flipped = flipped;
    }

    /**
     * Moves the robot given the speed to move forward/back and left/right
     *
     * @param powerY  the speed to move forward/back, -1 to 1, positive being forward
     * @param powerX  the speed to move left/right, -1 to 1,  positive being to the right
     * @param turn    the speed to turn at, -1 to 1, positive being clockwise
     * @param speed   a multiplier on the final speed
     * @param verbose whether or not to log extra data to telemetry
     */
    public void move(double powerY, double powerX, double turn, double speed, boolean verbose) {
        if (flipped) {
            powerX *= -1;
            turn *= -1;
        }

        double flPower = (powerY + turn + powerX) * speed;
        double frPower = (powerY - turn - powerX) * speed;
        double blPower = (powerY + turn - powerX) * speed;
        double brPower = (powerY - turn + powerX) * speed;

        double scale = Math.max(1, (Math.abs(powerY) + Math.abs(turn) + Math.abs(powerX)) * Math.abs(speed)); // shortcut for max(abs([fl,fr,bl,br]))
        flPower /= scale;
        frPower /= scale;
        blPower /= scale;
        brPower /= scale;

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);

        if (verbose) api.print(
                "Front Left: " + flPower + System.lineSeparator() +
                        "Back Left: " + blPower + System.lineSeparator() +
                        "Front Right: " + frPower + System.lineSeparator() +
                        "Back Right: " + brPower + System.lineSeparator()
        );
    }

    /**
     * Moves the robot given the speed to move forward/back and left/right
     * @param powerY the speed to move forward/back, -1 to 1, positive being forward
     * @param powerX the speed to move left/right, -1 to 1,  positive being to the right
     * @param turn   the speed to turn at, -1 to 1, positive being clockwise
     * @param speed  a multiplier on the final speed
     *
     */
    public void move(double powerY, double powerX, double turn, double speed) {
        move(powerY, powerX, turn, speed, false);
    }

    /**
     * Moves the robot in a given direction with a given speed
     *
     * @param direction the direction to move in, in degrees, with positive being left of forward
     * @param speed     the speed to move at
     * @param verbose   whether or not to log extra data to telemetry
     */
    public void move(double direction, double speed, boolean verbose) {
        move(Math.cos(Math.toRadians(direction)), Math.sin(Math.toRadians(-direction)), 0, speed, verbose);
    }

    /**
     * Moves the robot in a given direction with a given speed
     *
     * @param direction the direction to move in, in degrees, with positive being left of forward
     * @param speed     the speed to move at
     */
    public void move(double direction, double speed) {
        move(direction, speed, false);
    }

    /**
     * Stops the robot
     */
    public void stop() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}

package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PIDController;

public class MovementAPI {
    private final API api;

    private final DcMotor fl;
    private final DcMotor fr;
    private final DcMotor bl;
    private final DcMotor br;

    private static final double ANGLE_THRESHOLD = 1;
    // TODO: tune these values
    private static final double TURN_KP = 0;
    private static final double TURN_KI = 0;
    private static final double TURN_KD = 0;

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
     * <br><br>
     * <b>THIS IS FIELD CENTRIC - ROBOT WILL MOVE IN THIS DIRECTION NO MATTER WHAT THE HEADING IS</b>
     * <b>THIS IS A BREAKING CHANGE - PREVIOUS VERSIONS OF THIS API DID NOT INCLUDE THIS BEHAVIOR</b>
     *
     * @param powerY  the speed to move forward/back, -1 to 1, positive being forward
     * @param powerX  the speed to move left/right, -1 to 1,  positive being to the left
     * @param turn    the speed to turn at, -1 to 1, positive being clockwise
     * @param speed   a multiplier on the final speed
     * @param verbose whether or not to log extra data to telemetry
     */
    public void move(double powerY, double powerX, double turn, double speed, boolean verbose) {
        if (flipped) {
            powerX *= -1;
            turn *= -1;
        }

        double flPower = (powerY - powerX + turn) * speed;
        double frPower = (powerY + powerX - turn) * speed;
        double blPower = (powerY + powerX + turn) * speed;
        double brPower = (powerY - powerX - turn) * speed;

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
     * Moves the robot in a given direction with a given speed
     *
     * @param direction the direction to move in, in degrees, with positive being left of forward
     * @param speed     the speed to move at
     */
    public void move(double direction, double speed) {
        move(Math.cos(Math.toRadians(direction)), Math.sin(Math.toRadians(-direction)), 0, speed, false);
    }

    /**
     * Moves the direction in a given direction with a given speed, while also turning.
     * @param direction the direction to move in, in degrees, with positive being left of forward
     * @param turn      the speed to turn at, -1 to 1, positive being clockwise
     * @param speed     the speed to move at
     */
    public void move(double direction, double turn, double speed) {
        move(Math.cos(Math.toRadians(direction)), Math.sin(Math.toRadians(-direction)), turn, speed, false);
    }

    /**
     * Moves the robot in a given direction with a given speed for a given time.
     * While moving, the robot will attempt to maintain a constant heading, which is the
     * position the robot was at the last time {@link API#reset()} was called.
     * @see MovementAPI#move(double, double, double)
     * @see API#pause(double)
     * @param direction the direction to move in, in degrees, with positive being left of forward
     * @param speed     the speed to move at
     * @param seconds   how long to wait before stopping
     */
    public void moveFor(double direction, double speed, double seconds) {
        double time = api.opMode.getRuntime() + seconds;
        PIDController controller = new PIDController(TURN_KP, TURN_KI, TURN_KD, 0);
        while (api.opMode.getRuntime() < time) {
            double turn = controller.calculate(api.getHeading());
            move(direction, turn, speed);
        }
        stop();
    }

    /**
     * Turn to a given target at a given speed
     * The target is relative to the position of the robot the last time {@link API#reset()} was called.
     * @see API#reset()
     * @param target target heading, in degrees, within [-180, 180) with positive being counterclockwise
     * @param speed  the speed to move at
     */
    public void turnTo(double target, double speed) {
        double currentHeading = api.getHeading();
        PIDController controller = new PIDController(TURN_KP, TURN_KI, TURN_KD, target);
        while (Math.abs(currentHeading - target) > ANGLE_THRESHOLD) {
            currentHeading = api.getHeading();
            move(0, 0, controller.calculate(currentHeading), speed, false);
        }
        stop();
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

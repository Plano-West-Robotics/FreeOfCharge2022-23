package org.firstinspires.ftc.teamcode.kinematics;

import com.acmerobotics.dashboard.config.Config;

@Config
public enum LiftPosition {
    FLOOR, STACK, GROUND, LOW, MEDIUM, HIGH;

    public static int FLOOR_ENCODER_VAL = 10;
    public static int STACK_ENCODER_VAL = 600;
    public static int GROUND_ENCODER_VAL = 300;
    public static int LOW_ENCODER_VAL = 1000;
    public static int MEDIUM_ENCODER_VAL = 2000;
    public static int HIGH_ENCODER_VAL = 2900;

    LiftPosition() {

    }

    public int getEncoderVal() {
        switch (this) {
            case FLOOR:
                return FLOOR_ENCODER_VAL;
            case STACK:
                return STACK_ENCODER_VAL;
            case GROUND:
                return GROUND_ENCODER_VAL;
            case LOW:
                return LOW_ENCODER_VAL;
            case MEDIUM:
                return MEDIUM_ENCODER_VAL;
            case HIGH:
                return HIGH_ENCODER_VAL;
            default:
                return 0;
        }
    }

    public String toString() {
        switch (this) {
            case FLOOR:
                return "LiftPosition.FLOOR";
            case STACK:
                return "LiftPosition.STACK";
            case GROUND:
                return "LiftPosition.GROUND";
            case LOW:
                return "LiftPosition.LOW";
            case MEDIUM:
                return "LiftPosition.MEDIUM";
            case HIGH:
                return "LiftPosition.HIGH";
            default:
                return "uh... we dont know?";
        }
    }
}

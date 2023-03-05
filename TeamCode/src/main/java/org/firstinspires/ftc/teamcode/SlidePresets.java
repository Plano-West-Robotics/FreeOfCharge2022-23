package org.firstinspires.ftc.teamcode;

public enum SlidePresets {
    GROUND(0), LOW(1350), MEDIUM(2100), HIGH(3000), STACK_5(420);

    public final int position;

    SlidePresets(int position) {
        this.position = position;
    }
}

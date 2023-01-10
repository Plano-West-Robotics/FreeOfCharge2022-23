package org.firstinspires.ftc.teamcode;

public enum SlidePresets {
    GROUND(1), STACK_FIVE(688), LOW(1846), MEDIUM(2570), HIGH(3950);

    public final int position;

    SlidePresets(int position) {
        this.position = position;
    }
}

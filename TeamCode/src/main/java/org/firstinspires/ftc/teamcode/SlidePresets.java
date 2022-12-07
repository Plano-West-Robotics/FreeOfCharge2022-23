package org.firstinspires.ftc.teamcode;

public enum SlidePresets {
    // TODO: these are placeholders, measure the real positions then update this
    GROUND(0), LOW(100), MEDIUM(500), HIGH(1000);

    public final int position;

    SlidePresets(int position) {
        this.position = position;
    }
}

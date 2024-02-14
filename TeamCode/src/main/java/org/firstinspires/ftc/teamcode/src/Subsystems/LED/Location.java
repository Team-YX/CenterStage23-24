package org.firstinspires.ftc.teamcode.src.Subsystems.LED;

import androidx.annotation.NonNull;

public enum Location {
    LEFT,
    CENTER,
    RIGHT,
    UNKNOWN;

    @NonNull
    @Override
    public String toString() {
        switch (this) {
            case LEFT:
                return "LEFT";
            case CENTER:
                return "CENTER";
            case RIGHT:
                return "RIGHT";
            case UNKNOWN:
                return "UNKNOWN";
        }
        return "null";
    }
}
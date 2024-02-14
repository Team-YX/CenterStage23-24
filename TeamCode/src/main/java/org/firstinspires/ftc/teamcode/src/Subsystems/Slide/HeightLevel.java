package org.firstinspires.ftc.teamcode.src.Subsystems.Slide;

public enum HeightLevel {
    UnderGround,
    Down,
    HIGH_HIGH,
    HIGH_HIGH_HIGH,
    HIGH,
    FCones,
    TCones;


    /**
     * Pass in the Height Level, returns the the position to go to in ticks
     */
    //TODO: change the encoder heights
    private static int getEncoderCountFromLevel(HeightLevel level) {
        switch (level) {
            case HIGH:
                return 2920;
            case HIGH_HIGH_HIGH:
                return 3250;
            case HIGH_HIGH:
                return 3100;
            case FCones:
                return 211;
            case TCones:
                return 130;
            case UnderGround:
                return -100;
            case Down:
            default:
                return 0;
        }
    }

    public static int getEncoderCountFromEnum(HeightLevel level) {
        return getEncoderCountFromLevel(level);
    }

    private static final int[] heights;

    static {
        HeightLevel[] values = HeightLevel.values();
        heights = new int[values.length];
        for (int i = 0; i < values.length; i++) {
            heights[i] = HeightLevel.getEncoderCountFromEnum(values[i]);
        }
    }

    public static HeightLevel getClosestLevel(int currentLevel) {
        HeightLevel[] levels = HeightLevel.values();
        int[] distances = new int[levels.length];
        for (int i = 0; i < levels.length; i++) {
            distances[i] = Math.abs(heights[i] - currentLevel);
        }

        double smallestDistance = Double.MAX_VALUE;
        int indexOfSmallestValue = -1;

        for (int i = 0; i < levels.length; i++) {
            if (distances[i] < smallestDistance) {
                smallestDistance = distances[i];
                indexOfSmallestValue = i;
            }
        }
        return levels[indexOfSmallestValue];
    }

    private static int HeightLevelToInt(HeightLevel level) {
        switch (level) {

            case Down:
                return 0;
            case HIGH_HIGH:
                return 1;
            case HIGH_HIGH_HIGH:
                return 2;
            case HIGH:
                return 3;
        }
        return 0;
    }

    private static HeightLevel intToHeightLevel(int value) {
        switch (value) {
            case 0:
                return Down;
            case 1:
                return HIGH_HIGH;
            case 2:
                return HIGH_HIGH_HIGH;
            case 3:
                return HIGH;
        }

        if (value > 3) {
            return HIGH;
        }
        return Down;

    }

    public HeightLevel add(int amount) {
        return HeightLevel.intToHeightLevel(HeightLevelToInt(this) + amount);
    }

    public HeightLevel subtract(int amount) {
        return HeightLevel.intToHeightLevel(HeightLevelToInt(this) - amount);
    }


}

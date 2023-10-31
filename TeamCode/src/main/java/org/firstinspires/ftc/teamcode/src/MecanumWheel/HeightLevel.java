package org.firstinspires.ftc.teamcode.src.MecanumWheel;

public enum HeightLevel {
    UnderGround,
    Down,
    LowJunction,
    MediumJunction,
    HighJunction,
    FCones,
    TCones;


    /**
     * Pass in the Height Level, returns the the position to go to in ticks
     */
    private static int getEncoderCountFromLevel(HeightLevel level) {
        switch (level) {
            case HighJunction:
                return 1550;
            case MediumJunction:
                return 975;
            case LowJunction:
                return 575;
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
            case LowJunction:
                return 1;
            case MediumJunction:
                return 2;
            case HighJunction:
                return 3;
        }
        return 0;
    }

    private static HeightLevel intToHeightLevel(int value) {
        switch (value) {
            case 0:
                return Down;
            case 1:
                return LowJunction;
            case 2:
                return MediumJunction;
            case 3:
                return HighJunction;
        }

        if (value > 3) {
            return HighJunction;
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

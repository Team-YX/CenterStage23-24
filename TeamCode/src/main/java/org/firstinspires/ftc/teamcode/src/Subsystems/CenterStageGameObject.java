package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public enum CenterStageGameObject {
    GREEN_PIXEL,
    YELLOW_PIXEL,
    PURPLE_PIXEL,
    WHITE_PIXEL,
    EMPTY;

    //TODO fill out these rgb values
    private static final double[] GreenPixelRGB = new double[]{0, 0, 0};
    private static final double[] YellowPixelRGB = new double[]{0, 0, 0};
    private static final double[] PurplePixelRGB = new double[]{0, 0, 0};
    private static final double[] WhitePixelRGB = new double[]{0, 0, 0};
    private static final double[] EmptyRGB = new double[]{0, 0, 0};

    private static final CenterStageGameObject[] gameObjectArray = CenterStageGameObject.values();

    private static final double[][] RGBValuesOfEachItem;

    static {
        RGBValuesOfEachItem = new double[CenterStageGameObject.gameObjectArray.length][3];
        for (int x = 0; x < CenterStageGameObject.gameObjectArray.length; x++) {
            RGBValuesOfEachItem[x] = CenterStageGameObject.getRGBOfObject((CenterStageGameObject.gameObjectArray[x]));
        }
    }

    public static RevBlinkinLedDriver.BlinkinPattern getLEDColorFromItem(final CenterStageGameObject item) {
        switch (item) {
            case GREEN_PIXEL:
                return RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN;

            case YELLOW_PIXEL:
                return RevBlinkinLedDriver.BlinkinPattern.GOLD;

            case PURPLE_PIXEL:
                return RevBlinkinLedDriver.BlinkinPattern.VIOLET;

            case WHITE_PIXEL:
                return RevBlinkinLedDriver.BlinkinPattern.WHITE;

            case EMPTY:
                return RevBlinkinLedDriver.BlinkinPattern.BLACK;

            default:
                return null;
        }
    }

    private static double[] getRGBOfObject(final CenterStageGameObject item) {
        switch (item) {
            case GREEN_PIXEL:
                return GreenPixelRGB;

            case YELLOW_PIXEL:
                return YellowPixelRGB;

            case PURPLE_PIXEL:
                return PurplePixelRGB;

            case WHITE_PIXEL:
                return WhitePixelRGB;

            case EMPTY:
                return EmptyRGB;

            default:
                return null;
        }
    }

    public static CenterStageGameObject identify(double[] RGB) {

        final int numberOfGameElements = CenterStageGameObject.gameObjectArray.length;

        final double[] differences = new double[numberOfGameElements];

        for (int x = 0; x < numberOfGameElements; x++) {
            differences[x] = getDifferenceOfColor(RGB, RGBValuesOfEachItem[x]);
        }

        double smallestValue = Double.MAX_VALUE;
        int index = -1;

        for (int i = 0; i < numberOfGameElements; i++) {
            if (differences[i] < smallestValue) {
                smallestValue = differences[i];
                index = i;
            }
        }

        return CenterStageGameObject.gameObjectArray[index];
    }

    private static double getDifferenceOfColor(final double[] sight, final double[] object) {
        final double r = Math.abs(sight[0] - object[0]);
        final double g = Math.abs(sight[1] - object[1]);
        final double b = Math.abs(sight[2] - object[2]);

        return Math.sqrt(Math.pow(Math.sqrt(Math.pow(r, 2) + Math.pow(g, 2)), 2) + Math.pow(b, 2));
    }
}

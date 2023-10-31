package org.firstinspires.ftc.teamcode.src.MecanumWheel;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class IntiSlide extends LinearOpMode {

    public static final String RightLinearSlideMotorName = "RL";
    public static final String LeftLinearSlideMotorName = "LL";


    protected LinearSlide linearSlide_right;
    protected LinearSlide linearSlide_left;
    protected RevBlinkinLedDriver leds;
    protected RevColorSensorV3 InCol;

    protected void initLinearSlide() throws InterruptedException {
        {
            DcMotor tmp = hardwareMap.dcMotor.get(RightLinearSlideMotorName);
            tmp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            tmp.setPower(.1);
            Thread.sleep(1100);
            tmp.close();
        }
        linearSlide_right = new LinearSlide(hardwareMap, RightLinearSlideMotorName);
        linearSlide_left = new LinearSlide(hardwareMap, LeftLinearSlideMotorName);
    }

    protected void initLEDS() {
        leds = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
    }

    public void initInCol() {
        InCol = hardwareMap.get(RevColorSensorV3.class, "InCol");
        //    InCol.enableLed(true);
    }

}

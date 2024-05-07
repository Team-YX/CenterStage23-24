package org.firstinspires.ftc.teamcode.src.TeleOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "✝\uD83D\uDFE6BLUE_STATE_TELEOP\uD83D\uDFE6✝", group = "COMPETITION")
public class BlueTeleOp extends RedTeleOp{
    public BlueTeleOp() {
        super();
        this.defaultColor = BlinkinPattern.BLUE;
        this.currentBackPattern = this.defaultColor;
        this.currentFrontPattern = this.defaultColor;
    }
}

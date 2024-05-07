package org.firstinspires.ftc.teamcode.src.TeleOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.src.TeleOp.Templates.GenericOpmoodeTemplate;

import java.io.PrintWriter;
import java.io.StringWriter;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "✝NO_LED_STATE_TELEOP✝", group = "COMPETITION")
public class NoLEDTeleOp extends RedTeleOp {
    public NoLEDTeleOp() {
        super();
        this.defaultColor = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        this.currentBackPattern = this.defaultColor;
        this.currentFrontPattern = this.defaultColor;
    }
}
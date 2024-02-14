package org.firstinspires.ftc.teamcode.src.Subsystems.LED;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.TeleOp.Templates.GenericOpmoodeTemplate;

@TeleOp
public class LEDTest extends GenericOpmoodeTemplate {

    public void runOpMode() throws InterruptedException {

        leds.setPattern(BlinkinPattern.RED);

        waitForStart();
        while (opModeIsActive()) {

        }
    }
}

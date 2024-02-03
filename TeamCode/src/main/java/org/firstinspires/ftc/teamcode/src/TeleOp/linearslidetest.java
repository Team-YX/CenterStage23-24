package org.firstinspires.ftc.teamcode.src.TeleOp;

import android.transition.Slide;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Lift Test")
public class linearslidetest extends LinearOpMode {

    private LinearSlide Slide_right;
    private LinearSlide Slide_left;

    public void runOpMode() throws InterruptedException {
        Slide_left = new LinearSlide(hardwareMap, "RL");
        Slide_right = new LinearSlide(hardwareMap, "LL");

        Slide_right.reverseMotor();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                Slide_left.setTargetLevel(HeightLevel.HighJunction);
                Slide_right.setTargetLevel(HeightLevel.HighJunction);
            }
            if (gamepad1.dpad_down) {
                Slide_left.setTargetLevel(HeightLevel.MediumJunction);
                Slide_right.setTargetLevel(HeightLevel.MediumJunction);
            }
        }
    }
}
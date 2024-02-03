package org.firstinspires.ftc.teamcode.src.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LinearSlide {

    /**
     * The internal DcMotor object
     */

    final DcMotor Slide;

    /**
     * A constructor for the linear slide
     *
     * @param hardwareMap           The hardware map from the OpMode
     * @param slideName1,slideName2 The name of the slide
     */
    public LinearSlide(HardwareMap hardwareMap, String slideName1) {
        Slide = hardwareMap.dcMotor.get(slideName1);

        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setTargetPosition(HeightLevel.getEncoderCountFromEnum(HeightLevel.Down));
        Slide.setPower(1);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Sets the linear slide to go to the provided position
     *
     * @param targetPosition The position to go to in ticks
     */
    public void setTargetPosition(int targetPosition) {
        Slide.setTargetPosition(targetPosition);
    }

    /**
     * Sets the slide to go to the provided level
     *
     * @param level The level to go to
     */
    public void setTargetLevel(HeightLevel level) {
        Slide.setTargetPosition(HeightLevel.getEncoderCountFromEnum(level));
    }

    public void reverseMotor() {
        Slide.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
package org.firstinspires.ftc.teamcode.src.MecanumWheel;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

@Disabled
public class Slide {
    // Motor Objects //
    public DcMotorEx Slide;


    private List<DcMotorEx> slides;


    public enum Levels {
        High(-1550),
        Mid(-896),
        Low(-405),
        Ground(0);

        public final int pos;

        Levels(int pos) {
            this.pos = pos;
        }
    }


    public Slide(HardwareMap hardwareMap) {

        Slide = hardwareMap.get(DcMotorEx.class, "LR");

        Slide.setDirection(DcMotorSimple.Direction.REVERSE);


        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Slide.setTargetPosition(Levels.Ground.pos);


        Slide.setPower(-0.8);


        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }


    public void setTargetPosition(int targetPosition) {
        Slide.setTargetPosition(targetPosition);

    }

    public void setTargetLevel(Levels level) {
        Slide.setTargetPosition(level.pos);

    }

    public boolean isAtPosition() {
        return !(Slide.isBusy());
    }

    public boolean isAtPosition(int tolerance) {
        return Math.abs(Slide.getCurrentPosition() - Slide.getTargetPosition()) < tolerance;
    }

    public int getEncoderCount() {
        return Slide.getCurrentPosition();
    }


    public void waitOn() throws InterruptedException {
        do {
            Thread.sleep(40);
        } while (!this.isAtPosition(15));
    }

    public void goTo(Levels level) throws InterruptedException {
        setTargetLevel(level);

        do {
            Thread.sleep(40);
        } while (!this.isAtPosition(15));
    }

    public void goTo(int Position) throws InterruptedException {
        setTargetPosition(Position);

        do {
            Thread.sleep(40);
        } while (!this.isAtPosition(15));
    }

    public void reset() {
//        List<DcMotorEx> slides = Arrays.asList(leftSlide, rightSlide);
//
//        for (DcMotorEx slide : slides) {
//            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            slide.setTargetPosition(Levels.Ground.pos);
//            slide.setPower(1);
//            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }


        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Slide.setDirection(DcMotorSimple.Direction.REVERSE);

        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        Slide.setTargetPosition(Levels.Ground.pos);


        Slide.setPower(1);


        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void autoMode() {
        List<DcMotorEx> slides = Arrays.asList(Slide);

        for (DcMotorEx slide : slides) {
            slide.setTargetPosition(slide.getCurrentPosition());
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1);
        }
    }

    public void setMotorPower(double power) {
        Slide.setPower(power);
    }


    public int getTargetHeight() {
        return Slide.getTargetPosition();

    }

}

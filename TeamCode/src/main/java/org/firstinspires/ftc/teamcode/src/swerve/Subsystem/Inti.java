package org.firstinspires.ftc.teamcode.src.swerve.Subsystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class Inti extends LinearOpMode {

    public void INTI() {
        DcMotor front_right = hardwareMap.get(DcMotor.class, "FR");
        DcMotor front_left = hardwareMap.get(DcMotor.class, "FL");
        DcMotor back_left = hardwareMap.get(DcMotor.class, "BL");
        DcMotor back_right = hardwareMap.get(DcMotor.class, "BR");
        DcMotor linearSlide_right = hardwareMap.get(DcMotor.class, "RL");
        DcMotor linearSlide_left = hardwareMap.get(DcMotor.class, "LL");
        DcMotor IN_N_OUT = hardwareMap.get(DcMotor.class, "IN N OUT");

        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlide_left.setDirection(DcMotorSimple.Direction.REVERSE);

        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
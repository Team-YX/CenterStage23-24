package org.firstinspires.ftc.teamcode.src.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "MEET1_AUTO_TIME_Park", group = "Competition")
public class Time_Auto_Park extends LinearOpMode {

    DcMotor front_right;
    DcMotor front_left;
    DcMotor back_left;
    DcMotor back_right;
    DcMotor linearSlide_right;
    DcMotor linearSlide_left;
    DcMotor IN_N_OUT;

    public void runOpMode() throws InterruptedException {

        front_right = hardwareMap.get(DcMotor.class, "FR");
        front_left = hardwareMap.get(DcMotor.class, "FL");
        back_left = hardwareMap.get(DcMotor.class, "BL");
        back_right = hardwareMap.get(DcMotor.class, "BR");
        linearSlide_right = hardwareMap.get(DcMotor.class, "RL");
        linearSlide_left = hardwareMap.get(DcMotor.class, "LL");
        IN_N_OUT = hardwareMap.get(DcMotor.class, "IN N OUT");

        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlide_right.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide_left.setDirection(DcMotorSimple.Direction.FORWARD);

        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            front_left.setPower(0.5);
            front_right.setPower(0.5);
            back_left.setPower(0.5);
            back_right.setPower(0.5);
            sleep(2000);
            front_left.setPower(0);
            front_right.setPower(0);
            back_left.setPower(0);
            back_right.setPower(0);
            sleep(50000);
        }
    }

    public void FORWARD(double power, int time) {
        front_left.setPower(power);
        front_right.setPower(power);
        back_left.setPower(power);
        back_right.setPower(power);
        sleep(time);
    }

    public void STOP() {
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }
}

package org.firstinspires.ftc.teamcode.src.MecanumWheel;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TElEOP", group = "COMPETITION")
public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        ColorSensor csensor = hardwareMap.get(ColorSensor.class, "csensor");
        DistanceSensor dsensor2 = hardwareMap.get(DistanceSensor.class, "dsensor2");
        DistanceSensor dsensor1 = hardwareMap.get(DistanceSensor.class, "dsensor1");
        DcMotor front_right = hardwareMap.get(DcMotor.class, "FR");
        DcMotor front_left = hardwareMap.get(DcMotor.class, "FL");
        DcMotor back_left = hardwareMap.get(DcMotor.class, "BL");
        DcMotor back_right = hardwareMap.get(DcMotor.class, "BR");
        DcMotor linearSlide_right = hardwareMap.get(DcMotor.class, "RL");
        DcMotor linearSlide_left = hardwareMap.get(DcMotor.class, "LL");
        DcMotor IN_N_OUT = hardwareMap.get(DcMotor.class, "IN_N_OUT");

        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        linearSlide_right.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide_left.setDirection(DcMotorSimple.Direction.FORWARD);

        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double Speed = 1.0;
        double turnSpeed = 1;
        boolean aDepressed = false;

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive()) {
            // Distance Sensor Speed Adjuster
            if (dsensor1.getDistance(DistanceUnit.INCH) <= 4 ||
                    dsensor2.getDistance(DistanceUnit.INCH) <= 4) {
                Speed = 0.15;
            }

            //Basic Movement Control
            back_left.setPower(Speed * (-gamepad1.right_stick_x + (+gamepad1.left_stick_y - -gamepad1.left_stick_x)));
            back_right.setPower(Speed * (gamepad1.right_stick_x + +gamepad1.left_stick_y + -gamepad1.left_stick_x));
            front_right.setPower(Speed * 1 * (gamepad1.right_stick_x + (+gamepad1.left_stick_y - -gamepad1.left_stick_x)));
            front_left.setPower(Speed * 1 * (-gamepad1.right_stick_x + +gamepad1.left_stick_y + -gamepad1.left_stick_x));

            //Robot Control
            if (!gamepad1.a) {
                aDepressed = true;
            }
            if (gamepad1.a && aDepressed) {
                if (turnSpeed == 1) {
                    turnSpeed = 0.375;
                } else {
                    turnSpeed = 1;
                }
                aDepressed = false;
            }
            // Speed Control
            if (gamepad1.y) {
                Speed = 1;
            } else Speed = 0.6 + gamepad1.right_trigger / 2;

            //Intake Control
            if (gamepad2.b) {
                IN_N_OUT.setPower(0.8);
            } else if (gamepad2.x) {
                IN_N_OUT.setPower(-0.8);
            } else {
                IN_N_OUT.setPower(0);
            }
            //LinearSlide Control
            linearSlide_left.setPower(gamepad2.left_stick_y);
            linearSlide_right.setPower(gamepad2.left_stick_y);

            // Telemetry Data
            telemetry.addData("RUN", getRuntime());
            telemetry.addData("FR_Power", front_right.getPower());
            telemetry.addData("FLPower", front_left.getPower());
            telemetry.addData("BR_Power", back_right.getPower());
            telemetry.addData("BL_Power", back_left.getPower());
            telemetry.addData("SLide", linearSlide_left.getPower());
            telemetry.addData("IN_N_OUT", IN_N_OUT.getPower());
            telemetry.addData("Distance1", dsensor1.getDistance(DistanceUnit.INCH));
            telemetry.addData("Distance2", dsensor2.getDistance(DistanceUnit.INCH));
            telemetry.addData("Speed", Speed);
            telemetry.update();
        }
    }
}

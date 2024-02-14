package org.firstinspires.ftc.teamcode.src.TeleOp.Templates;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;



public abstract class OpmodeTemplate_Auto extends LinearOpMode {


    IMU imu;
//    protected Servo Intake1;
//    protected Servo Intake2;
    protected Servo Launcher;
    protected Servo plane_rotate;
    protected Servo extend_left;
    protected Servo extend_right;
    protected Servo Outtake_left;
    protected Servo Outtake_right;
    protected Servo OutDoor;
    protected CRServo InDoor;
    protected DcMotorEx front_right;
    protected DcMotorEx front_left;
    protected DcMotorEx back_left;
    protected DcMotorEx back_right;
    protected DcMotorEx IN_N_OUT;
    protected ColorRangeSensor back_color;
    protected ColorRangeSensor front_color;
//    protected RevBlinkinLedDriver leds;

    public void defaultInit() {

        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);
//        Intake1 = hardwareMap.get(Servo.class, "IntakeR");
//        Intake2 = hardwareMap.get(Servo.class, "IntakeL");
        extend_left = hardwareMap.get(Servo.class, "extendL");
        extend_right = hardwareMap.get(Servo.class, "extendR");
        Outtake_left = hardwareMap.get(Servo.class, "OuttakeL");
        Outtake_right = hardwareMap.get(Servo.class, "OuttakeR");
        plane_rotate = hardwareMap.get(Servo.class, "rotator");
        Launcher = hardwareMap.get(Servo.class, "Launcher");
        OutDoor = hardwareMap.get(Servo.class, "OutDoor");
        InDoor = hardwareMap.get(CRServo.class,"InDoor");

//      DistanceSensor dsensor2 = hardwareMap.get(DistanceSensor.class, "dsensor2");
//      DistanceSensor dsensor1 = hardwareMap.get(DistanceSensor.class, "dsensor1");

        front_right = hardwareMap.get(DcMotorEx.class, "FR");
        front_left = hardwareMap.get(DcMotorEx.class, "FL");
        back_left = hardwareMap.get(DcMotorEx.class, "BL");
        back_right = hardwareMap.get(DcMotorEx.class, "BR");
        IN_N_OUT = hardwareMap.get(DcMotorEx.class, "IN_N_OUT");
        back_color = hardwareMap.get(ColorRangeSensor.class, "back_color");
        front_color = hardwareMap.get(ColorRangeSensor.class, "front_color");

        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        OutDoor.setDirection(Servo.Direction.FORWARD);

        Outtake_left.setDirection(Servo.Direction.REVERSE);
        Outtake_right.setDirection(Servo.Direction.REVERSE);

        extend_right.setDirection(Servo.Direction.FORWARD);
        extend_left.setDirection(Servo.Direction.REVERSE);

        plane_rotate.setDirection(Servo.Direction.FORWARD);
        Launcher.setDirection(Servo.Direction.REVERSE);

        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        leds = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
    }


    public void Zero_power() {
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }

    void gamepadControlDriveTrain(double Speed) {
        back_left.setPower(Speed * (-gamepad1.right_stick_x + (+gamepad1.left_stick_y - -gamepad1.left_stick_x)));
        back_right.setPower(Speed * (gamepad1.right_stick_x + +gamepad1.left_stick_y + -gamepad1.left_stick_x));
        front_right.setPower(Speed * 1 * (gamepad1.right_stick_x + (+gamepad1.left_stick_y - -gamepad1.left_stick_x)));
        front_left.setPower(Speed * 1 * (-gamepad1.right_stick_x + +gamepad1.left_stick_y + -gamepad1.left_stick_x));
    }

    void IN_N_OUT_Control() {

        if (gamepad2.right_trigger > 0) {
            IN_N_OUT.setPower(-gamepad2.right_trigger);
        } else if (gamepad2.left_trigger > 0) {
            IN_N_OUT.setPower(gamepad2.left_trigger);
        } else {
            IN_N_OUT.setPower(0);
        }
    }

    void LauncherControl() {
        if (gamepad2.y) {
            Launcher.setPosition(0.5);
        } else if (gamepad2.a) {
            Launcher.setPosition(0.5);
        }
    }

//    void IntakeControl() {
//        if (gamepad2.dpad_down) {
//            Intake1.setPosition(0.565);
//            Intake2.setPosition(0.565);
//        }
//        if (gamepad2.dpad_up) {
//            Intake1.setPosition(0.3);
//            Intake2.setPosition(0.3);
//        }
//    }

    void TelemetryUpdate(double Speed) {
        telemetry.addData("RUN", getRuntime());
        telemetry.addData("FR_Power", front_right.getPower());
        telemetry.addData("FLPower", front_left.getPower());
        telemetry.addData("BR_Power", back_right.getPower());
        telemetry.addData("BL_Power", back_left.getPower());
        telemetry.addData("IN_N_OUT", IN_N_OUT.getPower());
//        telemetry.addData("Intake", Intake1.getPosition());
//                telemetry.addData("Distance1", dsensor1.getDistance(DistanceUnit.INCH));
//                telemetry.addData("Distance2", dsensor2.getDistance(DistanceUnit.INCH));
        telemetry.addData("Speed", Speed);
        telemetry.update();
    }

    public void Forward(double speed) {
        front_left.setPower(-speed);
        front_right.setPower(-speed);
        back_left.setPower(-speed);
        back_right.setPower(-speed);
    }

    public void Backward(double speed) {
        front_left.setPower(speed);
        front_right.setPower(speed);
        back_left.setPower(speed);
        back_right.setPower(speed);
    }

    public void Left(double speed) {
        front_left.setPower(speed);
        front_right.setPower(-speed);
        back_left.setPower(-speed);
        back_right.setPower(speed);
    }

    public void Right(double speed) {
        front_left.setPower(-speed);
        front_right.setPower(speed);
        back_left.setPower(speed);
        back_right.setPower(-speed);
    }

    public void Turn_Left(double speed) {
        front_left.setPower(-speed);
        front_right.setPower(speed);
        back_left.setPower(-speed);
        back_right.setPower(speed);
    }

    public void Turn_Right(double speed) {
        front_left.setPower(speed);
        front_right.setPower(-speed);
        back_left.setPower(speed);
        back_right.setPower(-speed);
    }

    public void Zero_power(double speed) {
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }

}

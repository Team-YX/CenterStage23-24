package org.firstinspires.ftc.teamcode.src.MecanumWheel;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class GenericOpmoodeTemplate extends LinearOpMode {


    IMU imu;
    protected Servo Intake1;
    protected Servo Intake2;
    protected CRServo Launcher;
    protected DcMotor front_right;
    protected DcMotor front_left;
    protected DcMotor back_left;
    protected DcMotor back_right;
    protected DcMotor linearSlide_right;
    protected DcMotor linearSlide_left;
    protected DcMotor IN_N_OUT;

    public void defaultInit() {

        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);
        Intake1 = hardwareMap.get(Servo.class, "Intake1");
        Intake2 = hardwareMap.get(Servo.class, "Intake2");
        Launcher = hardwareMap.get(CRServo.class, "Launcher");
//      DistanceSensor dsensor2 = hardwareMap.get(DistanceSensor.class, "dsensor2");
//      DistanceSensor dsensor1 = hardwareMap.get(DistanceSensor.class, "dsensor1");
        front_right = hardwareMap.get(DcMotor.class, "FR");
        front_left = hardwareMap.get(DcMotor.class, "FL");
        back_left = hardwareMap.get(DcMotor.class, "BL");
        back_right = hardwareMap.get(DcMotor.class, "BR");
        linearSlide_right = hardwareMap.get(DcMotor.class, "RL");
        linearSlide_left = hardwareMap.get(DcMotor.class, "LL");
        IN_N_OUT = hardwareMap.get(DcMotor.class, "IN_N_OUT");

        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        Intake1.setDirection(Servo.Direction.REVERSE);
        Intake2.setDirection(Servo.Direction.FORWARD);

        linearSlide_right.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide_left.setDirection(DcMotorSimple.Direction.FORWARD);

        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    void LinearSlideControl() {
        linearSlide_left.setPower(gamepad2.left_stick_y);
        linearSlide_right.setPower(gamepad2.left_stick_y);
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
            Launcher.setPower(1);
        } else if (gamepad2.a) {
            Launcher.setPower(-1);
        }
    }

    void IntakeControl() {
        if (gamepad2.dpad_down) {
            Intake1.setPosition(0.565);
            Intake2.setPosition(0.565);
        }
        if (gamepad2.dpad_up) {
            Intake1.setPosition(0.3);
            Intake2.setPosition(0.3);
        }
    }

    void TelemetryUpdate(double Speed) {
        telemetry.addData("RUN", getRuntime());
        telemetry.addData("FR_Power", front_right.getPower());
        telemetry.addData("FLPower", front_left.getPower());
        telemetry.addData("BR_Power", back_right.getPower());
        telemetry.addData("BL_Power", back_left.getPower());
        telemetry.addData("SLide", linearSlide_left.getPower());
        telemetry.addData("IN_N_OUT", IN_N_OUT.getPower());
        telemetry.addData("Intake", Intake1.getPosition());
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

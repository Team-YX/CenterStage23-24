package org.firstinspires.ftc.teamcode.src.MecanumWheel;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "MEET1_TElEOP", group = "Competition")
public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

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
            //Basic Movement Control
            back_left.setPower(Speed * (-gamepad1.right_stick_x + (+gamepad1.left_stick_y - -gamepad1.left_stick_x)));
            back_right.setPower(Speed * (gamepad1.right_stick_x + +gamepad1.left_stick_y + -gamepad1.left_stick_x));
            front_right.setPower(Speed * 1 * (gamepad1.right_stick_x + (+gamepad1.left_stick_y - -gamepad1.left_stick_x)));
            front_left.setPower(Speed * 1 * (-gamepad1.right_stick_x + +gamepad1.left_stick_y + -gamepad1.left_stick_x));

//            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//            double x = gamepad1.left_stick_x;
//            double rx = -gamepad1.right_stick_x;
//
//            // This button choice was made so that it is hard to hit on accident,
//            // it can be freely changed based on preference.
//            // The equivalent button is start on Xbox-style controllers.
//            if (gamepad1.options) {
//                imu.resetYaw();
//            }
//
//            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//            // Rotate the movement direction counter to the bot's rotation
//            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//            rotX = rotX * 1.1;  // Counteract imperfect strafing
//
//            // Denominator is the largest motor power (absolute value) or 1
//            // This ensures all the powers maintain the same ratio,
//            // but only if at least one is out of the range [-1, 1]
//            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//            double frontLeftPower = Speed * ((rotY + rotX + rx) / denominator);
//            double backLeftPower = Speed * ((rotY - rotX + rx) / denominator);
//            double frontRightPower = Speed * ((rotY - rotX - rx) / denominator);
//            double backRightPower = Speed * ((rotY + rotX - rx) / denominator);
//
//            front_left.setPower(frontLeftPower);
//            back_left.setPower(backLeftPower);
//            front_right.setPower(frontRightPower);
//            back_right.setPower(backRightPower);

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
            linearSlide_left.setPower(gamepad2.left_stick_y);
            linearSlide_right.setPower(gamepad2.left_stick_y);


            telemetry.addData("RUN", getRuntime());
            telemetry.addData("FR_Power", front_right.getPower());
            telemetry.addData("FLPower", front_left.getPower());
            telemetry.addData("BR_Power", back_right.getPower());
            telemetry.addData("BL_Power", back_left.getPower());
            telemetry.addData("SLide", linearSlide_left.getPower());
            telemetry.addData("IN_N_OUT", IN_N_OUT.getPower());
            telemetry.update();
        }
    }
}

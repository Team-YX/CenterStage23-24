package org.firstinspires.ftc.teamcode.src.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/* This program tests autonomous
 */
@Disabled
@Autonomous(name = "Encoder Auto")

public class Encoder extends LinearOpMode {

    // Motors
    public DcMotor Front_right;
    public DcMotor Front_left;
    public DcMotor backRight;
    public DcMotor backLeft;


    static final double COUNTS_PER_MOTOR_REV = 537;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.937;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    //calculations
    double diameterOfWheel = 3.96; //inches
    double circumference = diameterOfWheel * Math.PI;
    int distanceToGo = 12;
    double rotations = distanceToGo / circumference;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor front_right = hardwareMap.get(DcMotor.class, "FR");
        DcMotor front_left = hardwareMap.get(DcMotor.class, "FL");
        DcMotor back_left = hardwareMap.get(DcMotor.class, "BL");
        DcMotor back_right = hardwareMap.get(DcMotor.class, "BR");
        DcMotor linearSlide_right = hardwareMap.get(DcMotor.class, "RL");
        DcMotor linearSlide_left = hardwareMap.get(DcMotor.class, "LL");
        DcMotor IN_N_OUT = hardwareMap.get(DcMotor.class, "IN N OUT");

        //motors mirror each other

        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide_right.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide_left.setDirection(DcMotorSimple.Direction.FORWARD);

        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //encoders
//        Front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //wait for game to start
        waitForStart();

    }


    public void DriveBackwards(double power, int distance) {

        int movecounts = (int) (distance * COUNTS_PER_INCH);

        // reset encoders
        Front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //target position
        Front_left.setTargetPosition(-movecounts);
        Front_right.setTargetPosition(-movecounts);
        backRight.setTargetPosition(-movecounts);
        backLeft.setTargetPosition(-movecounts);

        //set mode
        Front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //set power
        Front_left.setPower(power);
        Front_right.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (opModeIsActive() && Front_left.isBusy() && Front_right.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            telemetry.addData("Driving Backwards", "Running to %7d", movecounts);
            telemetry.addData("Path2", "Running at %7d", Front_left.getCurrentPosition());
            telemetry.addData("Path3", "Running at %7d", Front_right.getCurrentPosition());
            telemetry.addData("Path4", "Running at %7d", backRight.getCurrentPosition());
            telemetry.addData("Path5", "Running at %7d", backLeft.getCurrentPosition());
            telemetry.update();

            idle();
        }
        StopDriving();
        Front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void DriveForward(double power, int distance) {

        //1240 = 1 rotation

        int movecounts = (int) (distance * COUNTS_PER_INCH);

        // reset encoders
        Front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //target position
        Front_left.setTargetPosition(movecounts);
        Front_right.setTargetPosition(movecounts);
        backRight.setTargetPosition(movecounts);
        backLeft.setTargetPosition(movecounts);

        Front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Front_left.setPower(power);
        Front_right.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);


//        if (Front_left.getCurrentPosition() >= distance || frontRight.getCurrentPosition() >= distance || backLeft.getCurrentPosition() >= distance || backRight.getCurrentPosition() >= distance) {
//            Front_left.setPower(0);
//            backLeft.setPower(0);
//            frontRight.setPower(0);
//            backRight.setPower(0);
//        }
//
//        if (Front_left.getCurrentPosition() >= 200) {
//            StopDriving();
//        }

        while (opModeIsActive() && Front_left.isBusy() && Front_right.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            telemetry.addData("Driving Forward", "Running to %7d", movecounts);
            telemetry.addData("Path2", "Running at %7d", Front_left.getCurrentPosition());
            telemetry.addData("Path3", "Running at %7d", Front_right.getCurrentPosition());
            telemetry.addData("Path4", "Running at %7d", backRight.getCurrentPosition());
            telemetry.addData("Path5", "Running at %7d", backLeft.getCurrentPosition());
            telemetry.update();

        }
        StopDriving();

        Front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void TurnRight(double power, int distance) {


        // reset encoders
        Front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //target position
        Front_left.setTargetPosition((int) (distance * 1.003));
        Front_right.setTargetPosition((int) (-distance * 1.008));
        backRight.setTargetPosition((int) (-distance * 1.001));
        backLeft.setTargetPosition((int) (distance * 1.008));

        Front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Front_left.setPower(power);
        Front_right.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (opModeIsActive() && Front_left.isBusy() && Front_right.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            telemetry.addData("Turning Right", "Running to %7d", distance);
            telemetry.addData("Path2", "Running at %7d", Front_left.getCurrentPosition());
            telemetry.addData("Path3", "Running at %7d", Front_right.getCurrentPosition());
            telemetry.addData("Path4", "Running at %7d", backRight.getCurrentPosition());
            telemetry.addData("Path5", "Running at %7d", backLeft.getCurrentPosition());
            telemetry.update();

            idle();
        }

        StopDriving();
        Front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void TurnLeft(double power, int distance) {


        // reset encoders
        Front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //target position
        Front_left.setTargetPosition(-distance);
        Front_right.setTargetPosition(distance);
        backRight.setTargetPosition(distance);
        backLeft.setTargetPosition(-distance);

        Front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // at 12 Volts battery, .4 speed, 1240 sleep time, turns approx. 90 deg

        Front_left.setPower(power);
        Front_right.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (opModeIsActive() && Front_left.isBusy() && Front_right.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            telemetry.addData("Turning Left", "Running to %7d", distance);
            telemetry.addData("Front Left", "Running at %7d", Front_left.getCurrentPosition());
            telemetry.addData("Front Right", "Running at %7d", Front_right.getCurrentPosition());
            telemetry.addData("Back Right", "Running at %7d", backRight.getCurrentPosition());
            telemetry.addData("Back Left", "Running at %7d", backLeft.getCurrentPosition());
            telemetry.update();

            idle();
        }

        Front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    public void StrafeRight(double power, int distance) {


        // reset encoders
        Front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //target position
        Front_left.setTargetPosition(-distance * 360);
        Front_right.setTargetPosition(distance * 360);
        backRight.setTargetPosition(-distance * 360);
        backLeft.setTargetPosition(distance * 360);

        Front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // at 12 Volts battery, .4 speed, 1240 sleep time, turns approx. 90 deg

        Front_left.setPower(0.85 * power);
        Front_right.setPower(0.85 * power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (opModeIsActive() && Front_left.isBusy() && Front_right.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            telemetry.addData("Strafing Right", "Running to %7d", distance);
            telemetry.addData("Front Left", "Running at %7d", Front_left.getCurrentPosition());
            telemetry.addData("Front Right", "Running at %7d", Front_right.getCurrentPosition());
            telemetry.addData("Back Right", "Running at %7d", backRight.getCurrentPosition());
            telemetry.addData("Back Left", "Running at %7d", backLeft.getCurrentPosition());
            telemetry.update();

            idle();
        }
        Front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void StrafeLeft(double power, int distance) {


        // reset encoders
        Front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //target position
        Front_left.setTargetPosition(distance * 360);
        Front_right.setTargetPosition(-distance * 360);
        backRight.setTargetPosition(distance * 360);
        backLeft.setTargetPosition(-distance * 360);

        Front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // at 12 Volts battery, .4 speed, 1240 sleep time, turns approx. 90 deg

        Front_left.setPower(0.85 * power);
        Front_right.setPower(0.85 * power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (opModeIsActive() && Front_left.isBusy() && Front_right.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            telemetry.addData("Strafing Left", "Running to %7d", distance);
            telemetry.addData("Front Left", "Running at %7d", Front_left.getCurrentPosition());
            telemetry.addData("Front Right", "Running at %7d", Front_right.getCurrentPosition());
            telemetry.addData("Back Right", "Running at %7d", backRight.getCurrentPosition());
            telemetry.addData("Back Left", "Running at %7d", backLeft.getCurrentPosition());
            telemetry.update();

            idle();
        }

        Front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void StopDriving() {
        Front_left.setPower(0);
        backLeft.setPower(0);
        Front_right.setPower(0);
        backRight.setPower(0);
    }

    private void Strafe_at_angle(double Power, double Angle) {
        double Distance = 0;
        double power_1;
        double power_2;

        Angle = Math.min(Math.max(Angle, 0), 360);
        power_1 = Math.cos((Angle + 45) / 180 * Math.PI);
        power_2 = Math.cos((Angle - 45) / 180 * Math.PI);
        power_1 = Power * power_1;
        power_2 = Power * power_2;
        Front_right.setPower(power_1);
        backLeft.setPower(power_1);
        Front_left.setPower(power_2);
        backRight.setPower(power_2);
        Front_right.setTargetPosition((int) (Distance * Math.sin(Angle / 180 * Math.PI) * 0.02120310002952885));
        Front_left.setTargetPosition((int) (Distance * Math.sin(Angle / 180 * Math.PI) * 0.02120310002952885));
        backRight.setTargetPosition((int) (Distance * Math.sin(Angle / 180 * Math.PI) * 0.02120310002952885));
        backLeft.setTargetPosition((int) (Distance * Math.sin(Angle / 180 * Math.PI) * 0.02120310002952885));
    }
}
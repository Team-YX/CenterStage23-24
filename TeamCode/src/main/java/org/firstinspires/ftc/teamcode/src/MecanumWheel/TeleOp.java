package org.firstinspires.ftc.teamcode.src.MecanumWheel;

import com.qualcomm.robotcore.util.RobotLog;

import java.io.PrintWriter;
import java.io.StringWriter;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "✝TELEOP✝", group = "COMPETITION")
public class TeleOp extends GenericOpmoodeTemplate {
    public static String getStackTraceAsString(Throwable e) {
        StringWriter sw = new StringWriter();
        PrintWriter pw = new PrintWriter(sw);
        e.printStackTrace(pw);
        return sw.toString();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            double Speed = 1;
            defaultInit();

            while (!isStarted() && !isStopRequested()) {
                Intake1.setPosition(0.3);
                Intake2.setPosition(0.3);
            }

            while (opModeIsActive()) {

                if (gamepad1.y) {
                    Speed = 1;
                } else Speed = 0.6 + gamepad1.right_trigger / 2;

                gamepadControlDriveTrain(Speed);

                IntakeControl();

                LauncherControl();

                IN_N_OUT_Control();

                LinearSlideControl();

                TelemetryUpdate(Speed);

            }
        } catch (Throwable t) {
            RobotLog.setGlobalErrorMsg(getStackTraceAsString(t));
        }
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
//      telemetry.addData("Distance1", dsensor1.getDistance(DistanceUnit.INCH));
//      telemetry.addData("Distance2", dsensor2.getDistance(DistanceUnit.INCH));
        telemetry.addData("Speed", Speed);
        telemetry.update();
    }
}


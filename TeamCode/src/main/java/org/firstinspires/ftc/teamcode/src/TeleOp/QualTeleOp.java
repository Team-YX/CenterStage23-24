package org.firstinspires.ftc.teamcode.src.TeleOp;

import com.qualcomm.robotcore.util.RobotLog;

import java.io.PrintWriter;
import java.io.StringWriter;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "✝QUAL_TELEOP✝", group = "COMPETITION")
public class QualTeleOp extends QUALGenericOpmoodeTemplate{

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
                Intake1.setPosition(0.265);
                Intake2.setPosition(0.245);
                plane_rotate.setPosition(0);
                Launcher.setPosition(0.34);
                Outtake_left.setPosition(0.575);
                Outtake_right.setPosition(0.445);
            }

            while (opModeIsActive()) {

                if (gamepad1.y) {
                    Speed = 1;
                } else Speed = 0.6 + gamepad1.right_trigger / 2;

                gamepadControlDriveTrain(Speed);

                IntakeControl();

                OuttakeControl();

                LauncherControl();

                ExtendControl();

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
        linearSlide_left.setPower(-gamepad2.left_stick_y);
        linearSlide_right.setPower(-gamepad2.left_stick_y);
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
        if (gamepad2.a) {
            plane_rotate.setPosition(0);
        } else if (gamepad2.y) {
            plane_rotate.setPosition(0.075);
        }
        if (gamepad2.b) {
            Launcher.setPosition(0.34);
        } else if (gamepad2.start) {
            Launcher.setPosition(0.2);
        }
    }

    void IntakeControl() {
        if (gamepad2.dpad_down) {
            Intake1.setPosition(.965);
            Intake2.setPosition(.965);
        }
        if (gamepad2.dpad_up) {
            Intake1.setPosition(.265);
            Intake2.setPosition(.245);
        }
    }

    void OuttakeControl() {
        if (gamepad2.left_bumper) {
            Outtake_left.setPosition(0.5);
            Outtake_right.setPosition(0.52);
        }
        if (gamepad2.right_bumper) {
            Outtake_left.setPosition(0.575);
            Outtake_right.setPosition(0.445);
        }
    }

    void ExtendControl(){
        if(gamepad2.dpad_right){
            extend_left.setPosition(0);
            extend_right.setPosition(0);
        }
        if(gamepad2.dpad_left){
            extend_left.setPosition(0.125);
            extend_right.setPosition(0.125);
        }
    }

    void TelemetryUpdate(double Speed) {
        telemetry.addData("RUN", getRuntime());

        telemetry.addData("inr", Intake1.getPosition());
        telemetry.addData("inl", Intake2.getPosition());
        telemetry.addData("outl", Outtake_left.getPosition());
        telemetry.addData("outr", Outtake_right.getPosition());
        telemetry.addData("lunch", Launcher.getPosition());
        telemetry.addData("rotor", plane_rotate.getPosition());
        telemetry.addData("exl", extend_left.getPosition());
        telemetry.addData("exr", extend_right.getPosition());

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
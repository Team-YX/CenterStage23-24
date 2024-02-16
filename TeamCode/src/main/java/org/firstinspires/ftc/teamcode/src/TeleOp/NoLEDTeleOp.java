package org.firstinspires.ftc.teamcode.src.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.src.TeleOp.Templates.GenericOpmoodeTemplate;

import java.io.PrintWriter;
import java.io.StringWriter;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "✝NO_LED_STATE_TELEOP✝", group = "COMPETITION")
public class NoLEDTeleOp extends GenericOpmoodeTemplate {

    public static String getStackTraceAsString(Throwable e) {
        StringWriter sw = new StringWriter();
        PrintWriter pw = new PrintWriter(sw);
        e.printStackTrace(pw);
        return sw.toString();
    }

    private final ElapsedTime IntakeTimer = new ElapsedTime();

    boolean Ydepressed = true;
    boolean Xdepresssed = true;
    boolean BackDepressed = true;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            double Speed = 1;
            defaultInit();

            while (!isStarted() && !isStopRequested()) {
                Outtake_left.setPosition(0.97);
                Outtake_right.setPosition(0.03);
                OutDoor.setPosition(0.4);
//                Intake1.setPosition(0.83);
//                Intake2.setPosition(0.83);
//                extend_left.setPosition(.435);
//                extend_right.setPosition(.45);
                plane_rotate.setPosition(0);
                Launcher.setPosition(0.425);
            }

            while (opModeIsActive()) {

                if (gamepad1.y) {
                    Speed = 1;
                } else Speed = 0.6 + gamepad1.right_trigger / 2;

                gamepadControlDriveTrain(Speed);

//                IntakeControl();

                OuttakeControl();

                LauncherControl();

//                ExtendControl();

                IN_N_OUT_Control();

                LinearSlideControl();

                Door();

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

    void Door() {
        if (!gamepad2.y) {
            Ydepressed = true;
        }
        if (gamepad2.y && Ydepressed) {
            if (Outtake_left.getPosition() > 0.6) {
                Outtake_left.setPosition(0.5);
                Outtake_right.setPosition(0.52);

                OutDoor.setPosition(0.1);
            } else {
                OutDoor.setPosition(0.4);
            }
            Ydepressed = false;
        }


//        } else if (gamepad1.x) {
//            InDoor.setPower(0.5);
//        } else {
//            InDoor.setPower(0);
//        }
    }

    void LinearSlideControl() {
        linearSlide_left.setPower(gamepad2.left_stick_y);
        linearSlide_right.setPower(-gamepad2.left_stick_y);
    }

    void IN_N_OUT_Control() {

        if (Math.abs(gamepad2.right_trigger - gamepad2.left_trigger) > 0.01) {
            IN_N_OUT.setPower(gamepad2.left_trigger - gamepad2.right_trigger);

            if (gamepad2.right_trigger > gamepad2.left_trigger && gamepad2.b) {
                OutDoor.setPosition(0.1);
                Outtake_left.setPosition(0.97);
                Outtake_right.setPosition(0.03);

                sleep(75);

                InDoor.setDirection(DcMotorSimple.Direction.FORWARD);
                InDoor.setPower(1);
            } else if (gamepad2.left_trigger > gamepad2.right_trigger && gamepad2.b) {
                InDoor.setPower(-1);
            } else {
                InDoor.setDirection(DcMotorSimple.Direction.FORWARD);
                InDoor.setPower(0);
            }
        } else {
            IN_N_OUT.setPower(0);
            InDoor.setPower(0);
        }
        if (IN_N_OUT.getCurrent(CurrentUnit.MILLIAMPS) >= 6000) {
            if (IntakeTimer.seconds() < 1) {
                IntakeTimer.seconds();
            }
        } else if (IN_N_OUT.getCurrent(CurrentUnit.MILLIAMPS) < 6000 &&
                Math.abs(gamepad2.right_trigger - gamepad2.left_trigger) > 0.01) {
            IntakeTimer.reset();
        }
    }

    void LauncherControl() {
        if (!gamepad2.x) {
            Xdepresssed = true;
        }
        if (gamepad2.x && Xdepresssed) {
            if (plane_rotate.getPosition() < 0.05) {
                plane_rotate.setPosition(0.09);//0.075
            } else {
                plane_rotate.setPosition(0);
            }
            Xdepresssed = false;
        }

        if (!gamepad2.back) {
            BackDepressed = true;
        }
        if (gamepad2.back && BackDepressed) {
            if (Launcher.getPosition() < 0.5) {
                Launcher.setPosition(0.6);
            } else {
                Launcher.setPosition(0.425);
            }
            BackDepressed = false;
        }
    }

    void OuttakeControl() {
        //Scoring
//        if (gamepad2.left_bumper) {
//            Outtake_left.setPosition(0.5);
//            Outtake_right.setPosition(0.52);
//        }
        if (gamepad2.a) {

            //Outtake
            OutDoor.setPosition(0.4);
            Outtake_left.setPosition(0.75);
            Outtake_right.setPosition(0.28);
        }

        if (gamepad2.right_bumper) {

            //Outtake Horizontal and door open
            Outtake_left.setPosition(0.97);
            Outtake_right.setPosition(0.03);
            OutDoor.setPosition(0.1);
        }
    }

    void ExtendControl() {
        //these two servos only operate between 0.15 and 0.85. Giving them a value outside that
        // range will cause them to stutter and glitch out
        if (gamepad2.dpad_right) {
            extend_left.setPosition(.435);
            extend_right.setPosition(.45);
        }
        if (gamepad2.dpad_left) {
            extend_left.setPosition(0.485);
            extend_right.setPosition(0.5);
        }
    }

    void TelemetryUpdate(double Speed) {
        telemetry.addData("RUN", getRuntime());

        telemetry.addData("outL", Outtake_left.getPosition());
        telemetry.addData("outR", Outtake_right.getPosition());
        telemetry.addData("lunch", Launcher.getPosition());
        telemetry.addData("rotor", plane_rotate.getPosition());
        telemetry.addData("exl", extend_left.getPosition());
        telemetry.addData("exr", extend_right.getPosition());
        telemetry.addData("OutDoor", OutDoor.getPosition());
        telemetry.addData("InDoor", InDoor.getPower());
        telemetry.addData("FR_Power", front_right.getPower());
        telemetry.addData("FLPower", front_left.getPower());
        telemetry.addData("BR_Power", back_right.getPower());
        telemetry.addData("BL_Power", back_left.getPower());
        telemetry.addData("SLide", linearSlide_left.getPower());
        telemetry.addData("IN_N_OUT", IN_N_OUT.getPower());
        telemetry.addData("IN_N_OUT", IN_N_OUT.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Speed", Speed);
        telemetry.update();
    }
}
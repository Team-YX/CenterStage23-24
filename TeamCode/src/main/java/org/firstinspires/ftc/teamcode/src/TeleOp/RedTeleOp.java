package org.firstinspires.ftc.teamcode.src.TeleOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.src.Subsystems.LED.CenterStageGameObject;
import org.firstinspires.ftc.teamcode.src.TeleOp.Templates.GenericOpmoodeTemplate;

import java.io.PrintWriter;
import java.io.StringWriter;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "✝\uD83D\uDFE5RED_STATE_TELEOP\uD83D\uDFE5✝", group = "COMPETITION")
public class RedTeleOp extends GenericOpmoodeTemplate {

    public static String getStackTraceAsString(Throwable e) {
        StringWriter sw = new StringWriter();
        PrintWriter pw = new PrintWriter(sw);
        e.printStackTrace(pw);
        return sw.toString();
    }

    private final ElapsedTime IntakeTimer = new ElapsedTime();
    private final ElapsedTime LEDTimer = new ElapsedTime();

    protected BlinkinPattern defaultColor;
    protected BlinkinPattern currentBackPattern;
    protected BlinkinPattern currentFrontPattern;
    protected BlinkinPattern proposedBackPattern;
    protected BlinkinPattern proposedFrontPattern;

    public RedTeleOp() {
        defaultColor = BlinkinPattern.RED;
        currentBackPattern = defaultColor;
        currentFrontPattern = defaultColor;
    }

    boolean Ydepressed = true;
    boolean Xdepresssed = true;
    boolean BackDepressed = true;
    boolean rbumpdepresesd = true;

    @Override
    public void runOpMode() throws InterruptedException {

        double Speed = 1;
        defaultInit();

        Outtake_left.setPosition(.86);
        Outtake_right.setPosition(0.14);
        OutDoor.setPosition(0.3);
//                Intake1.setPosition(0.83);
//                Intake2.setPosition(0.83);
        extend_left.setPosition(.825);
        extend_right.setPosition(.825);
        plane_rotate.setPosition(0);
        Launcher.setPosition(0.425);
        leds.setPattern(currentFrontPattern);

        waitForStart();


        while (opModeIsActive()) {
            if (gamepad1.y) {
                Speed = 1;
            } else Speed = 0.6 + gamepad1.right_trigger / 2;

            gamepadControlDriveTrain(Speed);

//            IntakeControl();

            OuttakeControl();

            LauncherControl();

            ExtendControl();

            IN_N_OUT_Control();

            LinearSlideControl();

            Door();

            LEDControl();

            TelemetryUpdate(Speed);


        }
    }

    void gamepadControlDriveTrain(double Speed) {
        back_left.setPower(Speed * (-gamepad1.right_stick_x + (+gamepad1.left_stick_y - -gamepad1.left_stick_x)));
        back_right.setPower(Speed * (gamepad1.right_stick_x + +gamepad1.left_stick_y + -gamepad1.left_stick_x));
        front_right.setPower(Speed * 1 * (gamepad1.right_stick_x + (+gamepad1.left_stick_y - -gamepad1.left_stick_x)));
        front_left.setPower(Speed * 1 * (-gamepad1.right_stick_x + +gamepad1.left_stick_y + -gamepad1.left_stick_x));
    }

    void Door() {
        if (gamepad2.y == false) {
            Ydepressed = true;
        }
        if (gamepad2.y && Ydepressed) {
            if (Outtake_left.getPosition() > 0.6) {
                Outtake_left.setPosition(0.5);
                Outtake_right.setPosition(0.5);

                OutDoor.setPosition(0.3);
            } else {
                OutDoor.setPosition(0.0);
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
                OutDoor.setPosition(0.0);
                Outtake_left.setPosition(.86);
                Outtake_right.setPosition(0.14);

                sleep(75);

                InDoor.setDirection(DcMotorSimple.Direction.FORWARD);
                InDoor.setPower(1);
            } else if (gamepad2.left_trigger > gamepad2.right_trigger && gamepad2.b) {
                InDoor.setDirection(DcMotorSimple.Direction.REVERSE);
                InDoor.setPower(1);
            } else {
                InDoor.setDirection(DcMotorSimple.Direction.FORWARD);
                InDoor.setPower(0);
            }
        } else {
            IN_N_OUT.setPower(0);
            InDoor.setPower(0);
        }

        if (IN_N_OUT.getCurrent(CurrentUnit.MILLIAMPS) >= 6000) {
            if (IntakeTimer.seconds() < 1 && IntakeTimer.seconds() > 0.25) {
                proposedBackPattern = BlinkinPattern.BLACK;
                proposedFrontPattern = BlinkinPattern.BLACK;
            }
        } else if (IN_N_OUT.getCurrent(CurrentUnit.MILLIAMPS) < 6000 &&
                Math.abs(gamepad2.right_trigger - gamepad2.left_trigger) > 0.01) {
            IntakeTimer.reset();
        }

    }

    void LauncherControl() {
        if (gamepad2.x == false) {
            Xdepresssed = true;
        }
        if (gamepad2.x && Xdepresssed) {
            if (plane_rotate.getPosition() < 0.05) {
                plane_rotate.setPosition(0.075);
            } else {
                plane_rotate.setPosition(0);
            }
            Xdepresssed = false;
        }

        if (gamepad2.back == false) {
            BackDepressed = true;
        }
        if (gamepad2.back && BackDepressed) {
            if (Launcher.getPosition() < 0.5) {
                Launcher.setPosition(1);
            } else {
                Launcher.setPosition(0.425);
            }
            BackDepressed = false;
        }
    }

//    void IntakeControl() {
//        if (gamepad2.dpad_down) {
//            Intake1.setPosition(.54);
//            Intake2.setPosition(.54);
//        }
//        if (gamepad2.dpad_up) {
//            Intake1.setPosition(.71);
//            Intake2.setPosition(.71);
//        }
//    }

    void OuttakeControl() {
        //Scoring
//        if (gamepad2.left_bumper) {
//            Outtake_left.setPosition(0.5);
//            Outtake_right.setPosition(0.52);
//        }
        if (gamepad2.a) {
            OutDoor.setPosition(0.3);
            Outtake_left.setPosition(0.77);
            Outtake_right.setPosition(0.23);
        }

        if (gamepad2.y == false) {
            Ydepressed = true;
        }
        if (gamepad2.y && Ydepressed) {
            if (Outtake_left.getPosition() > 0.6) {
                Outtake_left.setPosition(0.5);
                Outtake_right.setPosition(0.5);

                OutDoor.setPosition(0.3);
            } else {
                OutDoor.setPosition(0.0);
            }
            Ydepressed = false;
        }

        if (gamepad2.right_bumper == false) {
            rbumpdepresesd = true;
        }
        if (gamepad2.right_bumper && rbumpdepresesd) {
            if (OutDoor.getPosition() < .2) {
                //Horizontal
                Outtake_left.setPosition(.86);
                Outtake_right.setPosition(0.14);
                OutDoor.setPosition(0.3);
            } else {
                OutDoor.setPosition(0.0);
            }
            rbumpdepresesd = false;
        }


    }


    void ExtendControl() {
        //these two servos only operate between 0.15 and 0.85. Giving them a value outside that
        // range will cause them to stutter and glitch out
        if (gamepad2.dpad_right) {
            extend_left.setPosition(.825);
            extend_right.setPosition(.825);
        }
        if (gamepad2.dpad_left) {
            extend_left.setPosition(0.55);
            extend_right.setPosition(0.55);
        }
    }

    void LEDControl() {

        proposedBackPattern = CenterStageGameObject.getLEDColorFromItem(CenterStageGameObject.identify(getBackRGB()));
        proposedFrontPattern = CenterStageGameObject.getLEDColorFromItem(CenterStageGameObject.identify(getFrontRGB()));

        if (LEDTimer.seconds() > 0 && LEDTimer.seconds() < 1) {
            if (proposedBackPattern != null) {
                currentBackPattern = proposedBackPattern;
                leds.setPattern(currentBackPattern);
            } else if (proposedBackPattern == null) {
                currentBackPattern = defaultColor;
                leds.setPattern(currentBackPattern);
            }
        }
        if (LEDTimer.seconds() > 1.1 && LEDTimer.seconds() < 2) {
            if (proposedFrontPattern != null) {
                currentFrontPattern = proposedFrontPattern;
                leds.setPattern(currentFrontPattern);
            } else if (proposedFrontPattern == null) {
                currentFrontPattern = defaultColor;
                leds.setPattern(currentFrontPattern);
            }
        }
        if (LEDTimer.seconds() > 2.1) {
            LEDTimer.reset();
        }
    }

    void TelemetryUpdate(double Speed) {
        telemetry.addData("RUN", getRuntime());

//        telemetry.addData("inr", Intake1.getPosition());
//        telemetry.addData("inl", Intake2.getPosition());
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
//      telemetry.addData("Distance1", dsensor1.getDistance(DistanceUnit.INCH));
//      telemetry.addData("Distance2", dsensor2.getDistance(DistanceUnit.INCH));
        telemetry.addData("Speed", Speed);
        telemetry.addData("LEDtime", LEDTimer.seconds());
        telemetry.update();
    }
}
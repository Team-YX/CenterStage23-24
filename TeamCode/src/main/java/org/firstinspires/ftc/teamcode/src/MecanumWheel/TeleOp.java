package org.firstinspires.ftc.teamcode.src.MecanumWheel;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.teamcode.SubAssemblyCode.LinearSlide.HeightLevel;
//import org.firstinspires.ftc.teamcode.SubAssemblyCode.LinearSlide.LinearSlide;
//import org.firstinspires.ftc.teamcode.SubAssemblyCode.initSubSystems;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends LinearOpMode {

    final BlinkinPattern defaultColor = BlinkinPattern.BLUE;
    //Jacob Code-
    private final ElapsedTime slideResetTimer = new ElapsedTime();
    private boolean resetSlide = false;

    boolean y_depressed2 = true;
    boolean dPadUpDepressed = true;
    boolean dPadDownDepressed = true;
    boolean dPadRightDepressed = true;
    boolean dPadLeftDepressed = true;

    boolean manualSlideControl = false;

//    HeightLevel currentLevel = HeightLevel.Down;


    private DcMotor Front_right;
    private DcMotor Front_left;
    private DcMotor Back_left;
    private DcMotor Back_right;
    //    private LinearSlide linearSlide_right; // requires class to be LinearSlide instead of DcMotor for Jacob's code
    private DcMotor linearSlide_left;
    public CRServo In_Right;
    public CRServo In_Left;
    public Servo Grabber;
    public RevColorSensorV3 InCol;

    BlinkinPattern currentColor = defaultColor;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() throws InterruptedException {


        double Speed = 0.5;
        double turnSpeed = 1;
        boolean aDepressed = false;

        Front_right = hardwareMap.get(DcMotor.class, "FR");
        Front_left = hardwareMap.get(DcMotor.class, "FL");
        Back_left = hardwareMap.get(DcMotor.class, "BL");
        Back_right = hardwareMap.get(DcMotor.class, "BR");
//        linearSlide_right = new LinearSlide(hardwareMap, linearSlideMotorName);
//        linearSlide_left = hardwareMap.get(DcMotor.class, "LL");
//        In_Right = hardwareMap.get(CRServo.class, "IRS");
//        In_Left = hardwareMap.get(CRServo.class, "ILS");
//        Grabber = hardwareMap.get(Servo.class, "Grabber");

        // Put initialization blocks here.

        Front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        Front_left.setDirection(DcMotorSimple.Direction.FORWARD);
        Back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        Back_right.setDirection(DcMotorSimple.Direction.REVERSE);
//        linearSlide_right.setDirection(DcMotorSimple.Direction.FORWARD); Jacob code
//        linearSlide_left.setDirection(DcMotorSimple.Direction.REVERSE);
//        In_Right.setDirection(CRServo.Direction.REVERSE);
//        In_Left.setDirection(CRServo.Direction.FORWARD);
//        Grabber.setPosition(0.4);


        Front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        linearSlide_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); Jacob Code
//        linearSlide_left.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.FLOAT));
//
//        InCol = hardwareMap.get(RevColorSensorV3.class, "InCol");
//        InCol.enableLed(true);
        //Jacob Code
//        this.initLinearSlide();
//        this.initLEDS();
////        this.initInCol();
//        linearSlide_right.autoMode();
//        linearSlide_right.reverseMotor();
//
//        leds.setPattern(currentColor);

        waitForStart();

        while (opModeIsActive()) {
            //Basic Movement Control
            Back_left.setPower(Speed * (-gamepad1.right_stick_x + (+gamepad1.left_stick_y - -gamepad1.left_stick_x)));
            Back_right.setPower(Speed * (gamepad1.right_stick_x + +gamepad1.left_stick_y + -gamepad1.left_stick_x));
            Front_right.setPower(Speed * 1 * (gamepad1.right_stick_x + (+gamepad1.left_stick_y - -gamepad1.left_stick_x)));
            Front_left.setPower(Speed * 1 * (-gamepad1.right_stick_x + +gamepad1.left_stick_y + -gamepad1.left_stick_x));

            // Change the Speed of the robot

//            if (!gamepad1.a) {
//                aDepressed = true;
//            }
//            if (gamepad1.a && aDepressed) {
//                if (turnSpeed == 1) {
//                    turnSpeed = 0.375;
//                } else {
//                    turnSpeed = 1;
//                }
//                aDepressed = false;
//            }
//
//            if (gamepad1.y) {
//                Speed = 1;
//            } else Speed = 0.35 + gamepad1.right_trigger / 2;
//
//            //Intake Control
//            if (gamepad2.b) {
//                In_Left.setPower(-0.8);
//                In_Right.setPower(-0.8);
//            } else if (gamepad2.x) {
//                In_Right.setPower(0.8);
//                In_Left.setPower(0.8);
//                //Grabber Control
//            } else if (gamepad2.y) {
//                Grabber.setPosition(0.3);
//                sleep(100);
//                Grabber.setPosition(0.4);
//                leds.setPattern(defaultColor);
//            } else {
//                Grabber.setPosition(0.4);
//                In_Left.setPower(0);
//                In_Right.setPower(0);
//            }
//            //Code for LEDs if cone in grabber
//            if (InCol.getDistance(DistanceUnit.CM) <= 2) {
//                currentColor = BlinkinPattern.WHITE;
//            } else {
//                currentColor = defaultColor;
//            }
//        }
            telemetry.addData("RUN", getRuntime());
//            telemetry.addData("InCol", Math.round(InCol.getDistance(DistanceUnit.CM)));
            telemetry.update();
        }
    }
}

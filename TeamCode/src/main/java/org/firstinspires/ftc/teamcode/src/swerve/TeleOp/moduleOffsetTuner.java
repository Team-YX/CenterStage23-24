package org.firstinspires.ftc.teamcode.src.swerve.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.src.swerve.Utility.myDcMotorEx;

@Disabled
@TeleOp
public class moduleOffsetTuner extends LinearOpMode {
    public void runOpMode() {
        final AnalogInput mod1E, mod2E;
        final myDcMotorEx mod1m1, mod1m2, mod2m1, mod2m2;


        mod1E = hardwareMap.get(AnalogInput.class, "mod1E");
        mod2E = hardwareMap.get(AnalogInput.class, "mod2E");

        mod1m1 = new myDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftTopMotor"));
        mod2m1 = new myDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightTopMotor"));

        double module1Adjust = 23, module2Adjust = 40;

        waitForStart();
        while (opModeIsActive()) {

            double mod1P = (mod1E.getVoltage() * 109.09090909) - module1Adjust;
            double mod2P = (mod2E.getVoltage() * 109.09090909) - module2Adjust;

            mod1m1.setPower(0.1 * gamepad1.left_stick_y);
            mod2m1.setPower(0.1 * gamepad1.right_stick_y);

            telemetry.addData("mod1P:", mod1P);
            telemetry.addData("mod2P:", mod2P);

            telemetry.update();
        }
    }
}

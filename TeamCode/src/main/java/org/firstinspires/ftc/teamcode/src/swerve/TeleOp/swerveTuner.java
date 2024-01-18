package org.firstinspires.ftc.teamcode.src.swerve.TeleOp;

//Import EVERYTHING we need

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.src.swerve.Subsystem.SwerveDrive;
import org.firstinspires.ftc.teamcode.src.swerve.maths.PIDcontroller;


@Config
@TeleOp(name = "swerveTuner", group = "Linear Opmode")
public class swerveTuner extends LinearOpMode {

    //    public static double Kp = 8, Kd = 0, Ki = 5, Kf = 0, Kl = 0.001;
    PIDcontroller headingPID = new PIDcontroller(8, 1, 0, 0, 0.001);
    public static double offset = 0;
    double headingOut, headingTarget = 0;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        //class to swerve the swerve
        SwerveDrive swerve = new SwerveDrive(telemetry, hardwareMap, true);

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);


        waitForStart();
        while (opModeIsActive()) {

            //Clear the cache for better loop times (bulk sensor reads)
            controlHub.clearBulkCache();

            if (opModeIsActive()) {
                headingOut = headingPID.pidOut(AngleUnit.normalizeRadians(headingTarget - swerve.getHeading() * (Math.PI / 180)));
            } else {
                headingOut = 0;
            }
            swerve.drive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x);


            telemetry.update();
        }
    }
}
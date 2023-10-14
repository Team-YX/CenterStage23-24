package org.firstinspires.ftc.teamcode.src.swerve.TeleOp;

//Import EVERYTHING we need

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.swerve.Subsystem.SwerveDrive;


@Config
@TeleOp(name = "swerveTuner", group = "Linear Opmode")
public class swerveTuner extends LinearOpMode {

    public static double Kp = 0, Kd = 0, Ki = 0, Kf = 0, Kl = 10;
    public static double offset = 0;

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

            swerve.drive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x);


            telemetry.update();
        }
    }
}
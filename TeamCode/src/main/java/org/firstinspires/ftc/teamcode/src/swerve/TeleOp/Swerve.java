package org.firstinspires.ftc.teamcode.src.swerve.TeleOp;

//Import EVERYTHING we need
//import com.acmerobotics.dashboard.config.Config;
//import com.outoftheboxrobotics.photoncore.PhotonCore;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.src.swerve.Subsystem.SwerveDrive;
import org.firstinspires.ftc.teamcode.src.swerve.maths.PIDcontroller;


@TeleOp(name = "Swerve")
public class Swerve extends LinearOpMode {

    enum cycleStates {
        MANUAL
    }

    cycleStates cyclestate = cycleStates.MANUAL;

    ElapsedTime timer = new ElapsedTime();

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        //class to swerve the swerve
        SwerveDrive swerve = new SwerveDrive(telemetry, hardwareMap, true);

        PIDcontroller headingPID = new PIDcontroller(6, 0, 5, 0, 0.1);
        double headingOut, headingTarget = 0;

        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        //Fast loop go brrr- DO NOT DELETE
        //PhotonCore.enable();
        //PhotonCore.experimental.setMaximumParallelCommands(8);

        waitForStart();
        while (opModeIsActive()) {

            controlHub.clearBulkCache();

            if (opModeIsActive()) {
                headingOut = headingPID.pidOut(AngleUnit.normalizeRadians(headingTarget - swerve.getHeading() * (Math.PI / 180)));
            } else {
                headingOut = 0;
            }

            swerve.drive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, headingOut + gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x);
            if (gamepad1.a) {
                swerve.resetIMU();
            }

            telemetry.addData("power:leftx", gamepad1.left_stick_x);
            telemetry.addData("power:lefty", gamepad1.left_stick_y);
            telemetry.addData("power:rightx", gamepad1.right_stick_x);
            telemetry.update();

        }
    }
}
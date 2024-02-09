package org.firstinspires.ftc.teamcode.src.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.src.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.src.Subsystems.Location;
import org.firstinspires.ftc.teamcode.src.Subsystems.Slide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.Subsystems.Slide.LinearSlide;
import org.firstinspires.ftc.teamcode.src.TeleOp.OpmodeTemplate_Auto;
import org.firstinspires.ftc.teamcode.src.vision.PipeLine_RED;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name = "✝\uD83D\uDFE5 Close-Corner \uD83D\uDFE5✝", group = "COMPETITION")

public class R_Close_Corner extends OpmodeTemplate_Auto {

    private OpenCvCamera webcam;
    Location lcr;
    private LinearSlide Slide_right;
    private LinearSlide Slide_left;


    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 176; // height of wanted camera resolution

    public static double borderLeftX = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY = 0.0;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY = 0.0;   //fraction of pixels from the bottom of the cam to skip

    // RED Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 160.0, 100.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);

    @Override
    public void runOpMode() {
//        RevBlinkinLedDriver.BlinkinPattern defaultColor;
//
//        defaultColor = RevBlinkinLedDriver.BlinkinPattern.BLACK;
//
//        leds.setPattern(defaultColor);

        Slide_left = new LinearSlide(hardwareMap, "RL");
        Slide_right = new LinearSlide(hardwareMap, "LL");

        Slide_right.reverseMotor();

        defaultInit();

        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        //OpenCV Pipeline
        PipeLine_RED myPipeline;
        webcam.setPipeline(myPipeline = new PipeLine_RED(borderLeftX, borderRightX, borderTopY, borderBottomY));

        // Configuration of Pipeline
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        // Only if you are using ftcdashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 10);
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {

            Outtake_left.setPosition(0.9);//0.86
            Outtake_right.setPosition(0.1);//0.14
//            Intake1.setPosition(0.83);//.54 down
//            Intake2.setPosition(0.83);
            plane_rotate.setPosition(0);
            Launcher.setPosition(0.34);
//            Outtake_left.setPosition(0.75);
//            Outtake_right.setPosition(0.28);
            OutDoor.setPosition(0.4);

            if (myPipeline.getRectMidpointX() > 200 && myPipeline.getRectArea() > 1000) {
                lcr = Location.RIGHT;
            } else if (myPipeline.getRectMidpointX() > 110 && myPipeline.getRectMidpointX() < 200 && myPipeline.getRectArea() > 1000) {
                lcr = Location.CENTER;
            } else if (myPipeline.getRectMidpointX() < 110 && myPipeline.getRectArea() > 1000) {
                lcr = Location.LEFT;
            } else {
                lcr = Location.UNKNOWN;
                //return;
            }
            telemetry.addLine(lcr.toString());
            telemetry.addData("RectArea: ", myPipeline.getRectArea());
            telemetry.addData("MidPoint", myPipeline.getRectMidpointX());
            telemetry.update();
        }

        myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);

        switch (lcr) {
            case UNKNOWN:
            case RIGHT: {
                TrajectorySequence To_Marker = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .back(3)
                        .splineTo(new Vector2d(-30, 18), Math.toRadians(90))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, 40, 14.96))
                        .addDisplacementMarker(() -> {
                            Intake1.setPosition(.54);
                            Intake2.setPosition(.54);
                        })
                        .addDisplacementMarker(() -> {
                            IN_N_OUT.setPower(-0.3);
                        })
                        .waitSeconds(1.5)
                        .build();
                TrajectorySequence To_BackBoard = drive.trajectorySequenceBuilder(To_Marker.end())
                        .addDisplacementMarker(() -> {
                            Intake1.setPosition(.75);
                            Intake2.setPosition(.75);
                        })
                        .addDisplacementMarker(() -> {
                            IN_N_OUT.setPower(0);
                        })
//                        .forward(3)
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, 40, 14.96))
                        .lineToConstantHeading(new Vector2d(-25, 35))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, 40, 14.96))
                        .addDisplacementMarker(() -> {
                            Slide_right.setTargetLevel(HeightLevel.MediumJunction);
                            Slide_left.setTargetLevel(HeightLevel.MediumJunction);
                        })
                        .build();
                TrajectorySequence Score1 = drive.trajectorySequenceBuilder(To_BackBoard.end())
                        .back(5)
                        .addDisplacementMarker(() -> {
                            Outtake_left.setPosition(0.75);
                            Outtake_right.setPosition(0.25);
                        })
                        .waitSeconds(1)
                        .build();
                TrajectorySequence Score2 = drive.trajectorySequenceBuilder(Score1.end())
                        .addDisplacementMarker(() -> {
                            OutDoor.setPosition(0.1);
                        })
                        .waitSeconds(1)
                        .build();
                TrajectorySequence Park = drive.trajectorySequenceBuilder(Score2.end())
                        .forward(5)
                        .addDisplacementMarker(() -> {
                            OutDoor.setPosition(0.4);
                        })
                        .addDisplacementMarker(() -> {
                            Outtake_left.setPosition(0.9);
                            Outtake_right.setPosition(0.1);
                        })
                        .addDisplacementMarker(() -> {
                            Slide_right.setTargetLevel(HeightLevel.Down);
                            Slide_left.setTargetLevel(HeightLevel.Down);
                        })
                        .lineToConstantHeading(new Vector2d(0, 30))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, 40, 14.96))
                        .back(15)
                        .build();


                drive.followTrajectorySequence(To_Marker);
                drive.followTrajectorySequence(To_BackBoard);
                drive.followTrajectorySequence(Score1);
                drive.followTrajectorySequence(Score2);
                drive.followTrajectorySequence(Park);
                break;

            }
            case CENTER: {
                TrajectorySequence To_Marker = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .back(3)
                        .splineTo(new Vector2d(-40, 18), Math.toRadians(90))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, 40, 14.96))
                        .addDisplacementMarker(() -> {
                            Intake1.setPosition(.54);
                            Intake2.setPosition(.54);
                        })
                        .addDisplacementMarker(() -> {
                            IN_N_OUT.setPower(-0.3);
                        })
                        .waitSeconds(1.5)
                        .build();
                TrajectorySequence To_BackBoard = drive.trajectorySequenceBuilder(To_Marker.end())
                        .addDisplacementMarker(() -> {
                            Intake1.setPosition(.75);
                            Intake2.setPosition(.75);
                        })
                        .addDisplacementMarker(() -> {
                            IN_N_OUT.setPower(0);
                        })
                        .forward(10)
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, 40, 14.96))
//                        .addDisplacementMarker(() -> {
//                            Slide.setTargetLevel(HeightLevel.LowJunction);
//                        })
                        .build();
                TrajectorySequence Score1 = drive.trajectorySequenceBuilder(To_BackBoard.end())
                        .addDisplacementMarker(() -> {
                            Outtake_left.setPosition(0.5);
                            Outtake_right.setPosition(0.52);
                        })
                        .waitSeconds(1)
                        .build();
                TrajectorySequence Score2 = drive.trajectorySequenceBuilder(Score1.end())
                        .addDisplacementMarker(() -> {
                            OutDoor.setPosition(0.4);
                        })
                        .waitSeconds(1)
                        .build();
                TrajectorySequence Park = drive.trajectorySequenceBuilder(Score2.end())
//                        .addDisplacementMarker(() -> {
//                            linearSlide_left.setPower(-0.3);
//                            linearSlide_right.setPower(-0.3);
//                        })
                        .addDisplacementMarker(() -> {
                            OutDoor.setPosition(0.1);
                        })
                        .addDisplacementMarker(() -> {
                            Outtake_left.setPosition(0.75);
                            Outtake_right.setPosition(0.28);
                        })
                        .forward(10)
                        .lineToLinearHeading(new Pose2d(-3, -30, Math.toRadians(90)))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 35, 15.30))
                        .back(15)
                        .build();

                drive.followTrajectorySequence(To_Marker);
                drive.followTrajectorySequence(To_BackBoard);
//                drive.followTrajectorySequence(Score1);
//                drive.followTrajectorySequence(Score2);
//                drive.followTrajectorySequence(Park);
                break;

            }
            case LEFT: {
                TrajectorySequence To_Marker = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .back(3)
                        .splineTo(new Vector2d(-30, 18), Math.toRadians(90))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, 40, 14.96))
                        .addDisplacementMarker(() -> {
                            Intake1.setPosition(.54);
                            Intake2.setPosition(.54);
                        })
                        .addDisplacementMarker(() -> {
                            IN_N_OUT.setPower(-0.3);
                        })
                        .waitSeconds(1.5)
                        .build();
                TrajectorySequence To_BackBoard = drive.trajectorySequenceBuilder(To_Marker.end())
                        .addDisplacementMarker(() -> {
                            Intake1.setPosition(.75);
                            Intake2.setPosition(.75);
                        })
                        .addDisplacementMarker(() -> {
                            IN_N_OUT.setPower(0);
                        })
                        .forward(20)
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, 40, 14.96))
                        .lineToConstantHeading(new Vector2d(-33, 35))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, 40, 14.96))
                        .addDisplacementMarker(() -> {
                            Slide_left.setTargetLevel(HeightLevel.LowJunction);
                            Slide_right.setTargetLevel(HeightLevel.HighJunction);
                        })
                        .build();
                TrajectorySequence Score1 = drive.trajectorySequenceBuilder(To_BackBoard.end())
                        .addDisplacementMarker(() -> {
                            Outtake_left.setPosition(0.5);
                            Outtake_right.setPosition(0.52);
                        })
                        .waitSeconds(1)
                        .build();
                TrajectorySequence Score2 = drive.trajectorySequenceBuilder(Score1.end())
                        .addDisplacementMarker(() -> {
                            OutDoor.setPosition(0.4);
                        })
                        .waitSeconds(1)
                        .build();
                TrajectorySequence Park = drive.trajectorySequenceBuilder(Score2.end())
                        .back(2)
//                        .addDisplacementMarker(() -> {
//                            linearSlide_left.setPower(-0.3);
//                            linearSlide_right.setPower(-0.3);
//                        })
                        .addDisplacementMarker(() -> {
                            OutDoor.setPosition(0.1);
                        })
                        .addDisplacementMarker(() -> {
                            Outtake_left.setPosition(0.75);
                            Outtake_right.setPosition(0.28);
                        })
                        .forward(10)
                        .lineToLinearHeading(new Pose2d(-3, -30, Math.toRadians(90)))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 35, 15.30))
                        .back(15)
                        .build();


                drive.followTrajectorySequence(To_Marker);
                drive.followTrajectorySequence(To_BackBoard);
//                drive.followTrajectorySequence(Score1);
//                drive.followTrajectorySequence(Score2);
//                drive.followTrajectorySequence(Park);
                break;
            }
        }
    }
}



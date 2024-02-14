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
import org.firstinspires.ftc.teamcode.src.Subsystems.LED.Location;
import org.firstinspires.ftc.teamcode.src.Subsystems.Slide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.Subsystems.Slide.LinearSlide;
import org.firstinspires.ftc.teamcode.src.TeleOp.Templates.OpmodeTemplate_Auto;
import org.firstinspires.ftc.teamcode.src.Vision.ColorDetection.PipeLine_RED;
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

            Outtake_left.setPosition(0.9);
            Outtake_right.setPosition(0.1);
            plane_rotate.setPosition(0);
            Launcher.setPosition(0.34);
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
                        .splineTo(new Vector2d(-30, 20), Math.toRadians(90))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, 40, 14.96))
                        .waitSeconds(1.5)
                        .build();
                TrajectorySequence Drop = drive.trajectorySequenceBuilder(To_Marker.end())
                        .addDisplacementMarker(() -> {
                            IN_N_OUT.setPower(0.45);
                        })
                        .waitSeconds(2.5)
                        .build();
                TrajectorySequence To_BackBoard = drive.trajectorySequenceBuilder(Drop.end())
                        .lineToConstantHeading(new Vector2d(-22, 35))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(42, 42, 14.96))
                        .addDisplacementMarker(() -> {
                            Slide_right.setTargetLevel(HeightLevel.HIGH_HIGH);
                            Slide_left.setTargetLevel(HeightLevel.HIGH_HIGH);
                        })
                        .addDisplacementMarker(() -> {
                            IN_N_OUT.setPower(0);
                        })
                        .build();
                TrajectorySequence Score1 = drive.trajectorySequenceBuilder(To_BackBoard.end())
                        .back(6)
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
                        .lineToConstantHeading(new Vector2d(1, 35))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, 40, 14.96))
                        .build();


                drive.followTrajectorySequence(To_Marker);
                drive.followTrajectorySequence(Drop);
                drive.followTrajectorySequence(To_BackBoard);
                drive.followTrajectorySequence(Score1);
                drive.followTrajectorySequence(Score2);
                drive.followTrajectorySequence(Park);
                break;

            }
            case CENTER: {
                TrajectorySequence To_Marker = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .back(3)
                        .splineTo(new Vector2d(-37.7, 15), Math.toRadians(90))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, 40, 14.96))
                        .waitSeconds(1.5)
                        .build();
                TrajectorySequence Drop = drive.trajectorySequenceBuilder(To_Marker.end())
                        .addDisplacementMarker(() -> {
                            IN_N_OUT.setPower(0.48);
                        })
                        .waitSeconds(3)
                        .build();
                TrajectorySequence To_BackBoard = drive.trajectorySequenceBuilder(Drop.end())
                        .lineToConstantHeading(new Vector2d(-28, 35))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, 40, 14.96))
                        .addDisplacementMarker(() -> {
                            Slide_left.setTargetLevel(HeightLevel.HIGH_HIGH_HIGH);
                            Slide_right.setTargetLevel(HeightLevel.HIGH_HIGH_HIGH);
                        })
                        .addDisplacementMarker(() -> {
                            IN_N_OUT.setPower(0);
                        })
                        .build();
                TrajectorySequence Score1 = drive.trajectorySequenceBuilder(To_BackBoard.end())
                        .addDisplacementMarker(() -> {
                            Outtake_left.setPosition(0.75);
                            Outtake_right.setPosition(0.25);
                        })
                        .back(7)
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
                        .lineToConstantHeading(new Vector2d(1, 35))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, 40, 14.96))
                        .build();

                drive.followTrajectorySequence(To_Marker);
                drive.followTrajectorySequence(Drop);
                drive.followTrajectorySequence(To_BackBoard);
                drive.followTrajectorySequence(Score1);
                drive.followTrajectorySequence(Score2);
                drive.followTrajectorySequence(Park);
                break;
            }
            case LEFT: {
                TrajectorySequence To_Marker = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .lineToConstantHeading(new Vector2d(-32, 2.5))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, 40, 14.96))
                        .turn(Math.toRadians(-95))
                        .waitSeconds(1.5)
                        .build();
                TrajectorySequence Drop = drive.trajectorySequenceBuilder(To_Marker.end())
                        .addDisplacementMarker(() -> {
                            IN_N_OUT.setPower(.5);
                        })
                        .waitSeconds(3)
                        .build();
                TrajectorySequence To_BackBoard = drive.trajectorySequenceBuilder(Drop.end())
                        .lineToConstantHeading(new Vector2d(-34.5, 35))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, 40, 14.96))
                        .addDisplacementMarker(() -> {
                            Slide_left.setTargetLevel(HeightLevel.HIGH_HIGH);
                            Slide_right.setTargetLevel(HeightLevel.HIGH_HIGH);
                        })
                        .addDisplacementMarker(() -> {
                            IN_N_OUT.setPower(0);
                        })
                        .build();
                TrajectorySequence Score1 = drive.trajectorySequenceBuilder(To_BackBoard.end())
                        .back(7)
                        .addDisplacementMarker(() -> {
                            Outtake_left.setPosition(0.75);
                            Outtake_right.setPosition(0.25);
                        })
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
                        .lineToConstantHeading(new Vector2d(1, 35))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, 40, 14.96))
                        .build();


                drive.followTrajectorySequence(To_Marker);
                drive.followTrajectorySequence(Drop);
                drive.followTrajectorySequence(To_BackBoard);
                drive.followTrajectorySequence(Score1);
                drive.followTrajectorySequence(Score2);
                drive.followTrajectorySequence(Park);
                break;
            }
        }
    }
}



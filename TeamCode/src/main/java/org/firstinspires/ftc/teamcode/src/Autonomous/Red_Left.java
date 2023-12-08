package org.firstinspires.ftc.teamcode.src.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.src.MecanumWheel.GenericOpmoodeTemplate;
import org.firstinspires.ftc.teamcode.src.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.src.vision.PipeLine_RED;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name = "✝️\uD83D\uDFE5 Red_Left(5 Stack)\uD83D\uDFE5✝️", group = "COMPETITION")

public class Red_Left extends GenericOpmoodeTemplate {
    private OpenCvCamera webcam;
    int lcr;
    boolean Go;

    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 180; // height of wanted camera resolution

    public static double borderLeftX = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY = 0.0;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY = 0.0;   //fraction of pixels from the bottom of the cam to skip

    // RED Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 160.0, 100.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);

    @Override
    public void runOpMode() {
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
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
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

            Launcher.setPower(-1);
            Intake1.setPosition(0.3);
            Intake2.setPosition(0.3);

            if (myPipeline.getRectMidpointX() > 200 && myPipeline.getRectArea() > 1000) {
                telemetry.addLine("RIGHT");
                telemetry.addLine("");
                lcr = 1;
                Go = true;
            } else if (myPipeline.getRectMidpointX() > 110 && myPipeline.getRectMidpointX() < 200 && myPipeline.getRectArea() > 1000) {
                telemetry.addLine("CENTER");
                telemetry.addLine("");
                lcr = 2;
                Go = true;
            } else if (myPipeline.getRectMidpointX() < 110 && myPipeline.getRectArea() > 1000) {
                telemetry.addLine("LEFT");
                telemetry.addLine("");
                lcr = 3;
                Go = true;
            } else {
                telemetry.addLine("No prop found, please readjust the camera position");
                telemetry.addLine("");
                Go = false;
            }
            telemetry.addData("RectArea: ", myPipeline.getRectArea());
            telemetry.addData("MidPoint", myPipeline.getRectMidpointX());

            telemetry.addData("Status", Go);
            telemetry.update();
        }

        myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);

        if (Go) {
            Launcher.setPower(-1);
            Intake1.setPosition(0.3);
            Intake2.setPosition(0.3);
            //LEFT
            if (lcr == 1) {
                //RIGHT
                TrajectorySequence To_Marker = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .splineTo(new Vector2d(33, -5), Math.toRadians(270))
                        .back(7)
                        .build();
                TrajectorySequence Middle = drive.trajectorySequenceBuilder(To_Marker.end())
                        .lineToLinearHeading(new Pose2d(58, 3.5, Math.toRadians(270)))
                        .forward(65)
                        .build();
                TrajectorySequence To_BackBoard = drive.trajectorySequenceBuilder(Middle.end())
                        .lineToConstantHeading(new Vector2d(29, -95))
                        .build();
                TrajectorySequence Score = drive.trajectorySequenceBuilder(To_BackBoard.end())
                        .addDisplacementMarker(() -> {
                            IN_N_OUT.setPower(0.9);
                        })
                        .waitSeconds(3)
                        .build();
                TrajectorySequence Park = drive.trajectorySequenceBuilder(Score.end())
                        .addDisplacementMarker(() -> {
                            IN_N_OUT.setPower(0);
                        })
                        .back(10)
                        .lineToLinearHeading(new Pose2d(3, -85, Math.toRadians(273)))
                        .forward(15)
                        .build();

                drive.followTrajectorySequence(To_Marker);
                drive.followTrajectorySequence(Middle);
                drive.followTrajectorySequence(To_BackBoard);
                drive.followTrajectorySequence(Score);
                drive.followTrajectorySequence(Park);


                //CENTER
            } else if (lcr == 2) {
                TrajectorySequence To_Marker = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .lineToConstantHeading(new Vector2d(33, 0))
                        .back(10)
                        .strafeLeft(13)
                        .forward(20)
                        .build();
                TrajectorySequence Middle = drive.trajectorySequenceBuilder(To_Marker.end())
                        .lineToLinearHeading(new Pose2d(58, 3.5, Math.toRadians(270)))
                        .forward(65)
                        .build();
                TrajectorySequence To_BackBoard = drive.trajectorySequenceBuilder(Middle.end())
                        .lineToConstantHeading(new Vector2d(34, -95))
                        .build();
                TrajectorySequence Score = drive.trajectorySequenceBuilder(To_BackBoard.end())
                        .addDisplacementMarker(() -> {
                            IN_N_OUT.setPower(0.9);
                        })
                        .waitSeconds(3)
                        .build();
                TrajectorySequence Park = drive.trajectorySequenceBuilder(Score.end())
                        .addDisplacementMarker(() -> {
                            IN_N_OUT.setPower(0);
                        })
                        .back(10)
                        .lineToLinearHeading(new Pose2d(3, -85, Math.toRadians(273)))
                        .forward(15)
                        .build();

                drive.followTrajectorySequence(To_Marker);
                drive.followTrajectorySequence(Middle);
                drive.followTrajectorySequence(To_BackBoard);
                drive.followTrajectorySequence(Score);
                drive.followTrajectorySequence(Park);

                //LEFT
            } else if (lcr == 3) {
                TrajectorySequence To_Marker = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .lineToConstantHeading(new Vector2d(25, 12.5))
                        .back(10)
                        .strafeRight(13)
                        .build();
                TrajectorySequence Middle = drive.trajectorySequenceBuilder(To_Marker.end())
                        .lineToLinearHeading(new Pose2d(58, 3.5, Math.toRadians(270)))
                        .forward(65)
                        .build();
                TrajectorySequence To_BackBoard = drive.trajectorySequenceBuilder(Middle.end())
                        .lineToConstantHeading(new Vector2d(38, -95))
                        .build();
                TrajectorySequence Score = drive.trajectorySequenceBuilder(To_BackBoard.end())
                        .addDisplacementMarker(() -> {
                            IN_N_OUT.setPower(0.9);
                        })
                        .waitSeconds(3)
                        .build();
                TrajectorySequence Park = drive.trajectorySequenceBuilder(Score.end())
                        .addDisplacementMarker(() -> {
                            IN_N_OUT.setPower(0);
                        })
                        .back(10)
                        .lineToLinearHeading(new Pose2d(3, -85, Math.toRadians(273)))
                        .forward(15)
                        .build();

                drive.followTrajectorySequence(To_Marker);
                drive.followTrajectorySequence(Middle);
                drive.followTrajectorySequence(To_BackBoard);
                drive.followTrajectorySequence(Score);
                drive.followTrajectorySequence(Park);
            }
        }
    }
}



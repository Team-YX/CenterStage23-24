package org.firstinspires.ftc.teamcode.src.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.src.TeleOp.Location;
import org.firstinspires.ftc.teamcode.src.TeleOp.QUALGenericOpmoodeTemplate;
import org.firstinspires.ftc.teamcode.src.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.src.vision.PipeLine_BLUE;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name = "✝\uD83D\uDFE6 Blue_Right(5 Stack) \uD83D\uDFE6✝", group = "COMPETITION")

public class Blue_Right extends QUALGenericOpmoodeTemplate {
    private OpenCvCamera webcam;
    Location lcr;

    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 180; // height of wanted camera resolution

    public static double borderLeftX = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY = 0.0;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY = 0.0;   //fraction of pixels from the bottom of the cam to skip

    // RED Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 60, 150);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);

    @Override
    public void runOpMode() {
        defaultInit();

        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        //OpenCV Pipeline
        PipeLine_BLUE myPipeline;
        webcam.setPipeline(myPipeline = new PipeLine_BLUE(borderLeftX, borderRightX, borderTopY, borderBottomY));

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

            Intake1.setPosition(0.265);
            Intake2.setPosition(0.245);
            plane_rotate.setPosition(0);
            Launcher.setPosition(0.34);
            Outtake_left.setPosition(0.575);
            Outtake_right.setPosition(0.445);

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
            telemetry.addData("lcr", lcr);
            telemetry.update();
        }


        myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);

        switch (lcr) {
            case UNKNOWN:
            case RIGHT: {
                TrajectorySequence To_Marker = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .lineToConstantHeading(new Vector2d(25, -11))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 35, 11.03))
                        .back(10)
                        .strafeLeft(13)
                        .build();
                TrajectorySequence Middle = drive.trajectorySequenceBuilder(To_Marker.end())
                        .lineToLinearHeading(new Pose2d(58, -3.5, Math.toRadians(94)))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 35, 11.03))
                        .forward(65)
                        .waitSeconds(5)
                        .build();
                TrajectorySequence To_BackBoard = drive.trajectorySequenceBuilder(Middle.end())
                        .lineToConstantHeading(new Vector2d(31, 82.5))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 35, 11.03))
                        .forward(1.5)
                        .build();
                TrajectorySequence Score = drive.trajectorySequenceBuilder(To_BackBoard.end())
                        .addDisplacementMarker(() -> {
                            linearSlide_left.setPower(-0.5);
                            linearSlide_right.setPower(-0.5);
                        })
                        .addDisplacementMarker(() -> {
                            IN_N_OUT.setPower(0.9);
                        })
                        .waitSeconds(2)
                        .build();
                TrajectorySequence Park = drive.trajectorySequenceBuilder(Score.end())
                        .addDisplacementMarker(() -> {
                            linearSlide_left.setPower(0.15);
                            linearSlide_right.setPower(0.15);
                        })
                        .addDisplacementMarker(() -> {
                            IN_N_OUT.setPower(0);
                        })
                        .back(15)
                        .lineToConstantHeading(new Vector2d(-2, 76.5))
                        .forward(15)
                        .build();
                drive.followTrajectorySequence(To_Marker);
                drive.followTrajectorySequence(Middle);
                drive.followTrajectorySequence(To_BackBoard);
                drive.followTrajectorySequence(Score);
                drive.followTrajectorySequence(Park);
                break;
            }

            //CENTER
            case CENTER: {
                TrajectorySequence To_Marker = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .lineToConstantHeading(new Vector2d(33, 0))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 35, 11.03))
                        .back(10)
                        .strafeRight(13)
                        .forward(20)
                        .build();
                TrajectorySequence Middle = drive.trajectorySequenceBuilder(To_Marker.end())
                        .lineToLinearHeading(new Pose2d(58, -3.5, Math.toRadians(94)))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 35, 11.03))
                        .forward(65)
                        .build();
                TrajectorySequence To_BackBoard = drive.trajectorySequenceBuilder(Middle.end())
                        .lineToConstantHeading(new Vector2d(27, 82.5))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 35, 11.03))
                        .forward(1.5)
                        .waitSeconds(5)
                        .build();
                TrajectorySequence Score = drive.trajectorySequenceBuilder(To_BackBoard.end())
                        .addDisplacementMarker(() -> {
                            linearSlide_left.setPower(-0.5);
                            linearSlide_right.setPower(-0.5);
                        })
                        .addDisplacementMarker(() -> {
                            IN_N_OUT.setPower(0.9);
                        })
                        .waitSeconds(2)
                        .build();
                TrajectorySequence Park = drive.trajectorySequenceBuilder(Score.end())
                        .addDisplacementMarker(() -> {
                            linearSlide_left.setPower(0.15);
                            linearSlide_right.setPower(0.15);
                        })
                        .addDisplacementMarker(() -> {
                            IN_N_OUT.setPower(0);
                        })
                        .back(15)
                        .lineToConstantHeading(new Vector2d(-2, 76.5))
                        .forward(15)
                        .build();
                drive.followTrajectorySequence(To_Marker);
                drive.followTrajectorySequence(Middle);
                drive.followTrajectorySequence(To_BackBoard);
                drive.followTrajectorySequence(Score);
                drive.followTrajectorySequence(Park);
                break;
            }
            //LEFT
            case LEFT: {
                TrajectorySequence To_Marker = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .splineTo(new Vector2d(33, 5), Math.toRadians(90))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 35, 11.03))
                        .back(7)
                        .build();
                TrajectorySequence Middle = drive.trajectorySequenceBuilder(To_Marker.end())
                        .lineToLinearHeading(new Pose2d(58, -3.5, Math.toRadians(94)))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 35, 11.03))
                        .forward(65)
                        .waitSeconds(5)
                        .build();
                TrajectorySequence To_BackBoard = drive.trajectorySequenceBuilder(Middle.end())
                        .lineToConstantHeading(new Vector2d(18, 82.5))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 35, 11.03))
                        .forward(1.5)
                        .build();
                TrajectorySequence Score = drive.trajectorySequenceBuilder(To_BackBoard.end())
                        .addDisplacementMarker(() -> {
                            linearSlide_left.setPower(-0.5);
                            linearSlide_right.setPower(-0.5);
                        })
                        .addDisplacementMarker(() -> {
                            IN_N_OUT.setPower(0.9);
                        })
                        .waitSeconds(2)
                        .build();
                TrajectorySequence Park = drive.trajectorySequenceBuilder(Score.end())
                        .addDisplacementMarker(() -> {
                            linearSlide_left.setPower(0.15);
                            linearSlide_right.setPower(0.15);
                        })
                        .addDisplacementMarker(() -> {
                            IN_N_OUT.setPower(0);
                        })
                        .back(15)
                        .lineToConstantHeading(new Vector2d(-2, 76.5))
                        .forward(15)
                        .build();
                drive.followTrajectorySequence(To_Marker);
                drive.followTrajectorySequence(Middle);
                drive.followTrajectorySequence(To_BackBoard);
                drive.followTrajectorySequence(Score);
                drive.followTrajectorySequence(Park);
                break;
            }
        }
    }
}
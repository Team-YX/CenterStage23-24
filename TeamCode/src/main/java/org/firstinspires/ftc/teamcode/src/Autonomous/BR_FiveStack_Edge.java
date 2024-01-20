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
import org.firstinspires.ftc.teamcode.src.TeleOp.Location;
import org.firstinspires.ftc.teamcode.src.TeleOp.QUALGenericOpmoodeTemplate;
import org.firstinspires.ftc.teamcode.src.vision.PipeLine_BLUE;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name = "✝\uD83D\uDFE6 5 Stack-Edge \uD83D\uDFE6✝", group = "COMPETITION")

public class BR_FiveStack_Edge extends QUALGenericOpmoodeTemplate {
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

            Intake1.setPosition(0.73);
            Intake2.setPosition(0.73);
            plane_rotate.setPosition(0);
            Launcher.setPosition(0.34);
            Outtake_left.setPosition(0.75);
            Outtake_right.setPosition(0.28);
            OutDoor.setPosition(0.1);

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
                        .lineToConstantHeading(new Vector2d(26, -14))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 35, 15.30))
                        .back(10)
                        .strafeLeft(13)
                        .forward(25)
                        .build();
                TrajectorySequence Middle = drive.trajectorySequenceBuilder(To_Marker.end())
                        .lineToLinearHeading(new Pose2d(55, -3.5, Math.toRadians(83)))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 35, 15.30))
                        .forward(65)
                        .waitSeconds(0.5)
                        .build();
                TrajectorySequence To_BackBoard = drive.trajectorySequenceBuilder(Middle.end())
                        .lineToLinearHeading(new Pose2d(36, 86.5, Math.toRadians(260)))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, 40, 15.30))
                        .back(1.5)
                        .build();
                TrajectorySequence Score1 = drive.trajectorySequenceBuilder(To_BackBoard.end())
                        .addDisplacementMarker(() -> {
                            linearSlide_left.setPower(0.5);
                            linearSlide_right.setPower(0.5);
                        })
                        .waitSeconds(2.2)
                        .build();
                TrajectorySequence Score2 = drive.trajectorySequenceBuilder(Score1.end())
                        .addDisplacementMarker(() -> {
                            Outtake_left.setPosition(0.5);
                            Outtake_right.setPosition(0.52);
                        })
                        .waitSeconds(1)
                        .build();
                TrajectorySequence Score3 = drive.trajectorySequenceBuilder(Score2.end())
                        .addDisplacementMarker(() -> {
                            OutDoor.setPosition(0.4);
                        })
                        .waitSeconds(1)
                        .build();
                TrajectorySequence Park = drive.trajectorySequenceBuilder(Score3.end())
                        .addDisplacementMarker(() -> {
                            linearSlide_left.setPower(-0.3);
                            linearSlide_right.setPower(-0.3);
                        })
                        .addDisplacementMarker(() -> {
                            OutDoor.setPosition(0.1);
                        })
                        .addDisplacementMarker(() -> {
                            Outtake_left.setPosition(0.75);
                            Outtake_right.setPosition(0.28);
                        })
                        .forward(15)
                        .lineToLinearHeading(new Pose2d(54, 82, Math.toRadians(260)))
                        .back(15)
                        .build();
                drive.followTrajectorySequence(To_Marker);
                drive.followTrajectorySequence(Middle);
                drive.followTrajectorySequence(To_BackBoard);
                drive.followTrajectorySequence(Score1);
                drive.followTrajectorySequence(Score2);
                drive.followTrajectorySequence(Score3);
                drive.followTrajectorySequence(Park);
                break;
            }

            case CENTER: {
                TrajectorySequence To_Marker = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .lineToConstantHeading(new Vector2d(33, 0))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 35, 15.30))
                        .back(10)
                        .strafeRight(13)
                        .forward(20)
                        .build();
                TrajectorySequence Middle = drive.trajectorySequenceBuilder(To_Marker.end())
                        .lineToLinearHeading(new Pose2d(55, -3.5, Math.toRadians(90)))//58
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 35, 15.30))
                        .forward(65)
                        .build();
                TrajectorySequence To_BackBoard = drive.trajectorySequenceBuilder(Middle.end())
                        .lineToLinearHeading(new Pose2d(29.5, 86.5, Math.toRadians(260)))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, 40, 15.30))
                        .back(1.5)
                        .build();
                TrajectorySequence Score1 = drive.trajectorySequenceBuilder(To_BackBoard.end())
                        .addDisplacementMarker(() -> {
                            linearSlide_left.setPower(0.5);
                            linearSlide_right.setPower(0.5);
                        })
                        .waitSeconds(2.2)
                        .build();
                TrajectorySequence Score2 = drive.trajectorySequenceBuilder(Score1.end())
                        .addDisplacementMarker(() -> {
                            Outtake_left.setPosition(0.5);
                            Outtake_right.setPosition(0.52);
                        })
                        .waitSeconds(1)
                        .build();
                TrajectorySequence Score3 = drive.trajectorySequenceBuilder(Score2.end())
                        .addDisplacementMarker(() -> {
                            OutDoor.setPosition(0.4);
                        })
                        .waitSeconds(1)
                        .build();
                TrajectorySequence Park = drive.trajectorySequenceBuilder(Score3.end())
                        .addDisplacementMarker(() -> {
                            linearSlide_left.setPower(-0.3);
                            linearSlide_right.setPower(-0.3);
                        })
                        .addDisplacementMarker(() -> {
                            OutDoor.setPosition(0.1);
                        })
                        .addDisplacementMarker(() -> {
                            Outtake_left.setPosition(0.75);
                            Outtake_right.setPosition(0.28);
                        })
                        .forward(15)
                        .lineToLinearHeading(new Pose2d(54, 82, Math.toRadians(260)))
                        .back(15)
                        .build();
                drive.followTrajectorySequence(To_Marker);
                drive.followTrajectorySequence(Middle);
                drive.followTrajectorySequence(To_BackBoard);
                drive.followTrajectorySequence(Score1);
                drive.followTrajectorySequence(Score2);
                drive.followTrajectorySequence(Score3);
                drive.followTrajectorySequence(Park);
                break;
            }
            case LEFT: {
                TrajectorySequence To_Marker = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .splineTo(new Vector2d(26.5, 7), Math.toRadians(90))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 35, 15.30))
                        .back(10)
                        .build();
                TrajectorySequence Middle = drive.trajectorySequenceBuilder(To_Marker.end())
                        .lineToLinearHeading(new Pose2d(55, -3.5, Math.toRadians(80)))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 35, 15.30))
                        .forward(65)
                        .waitSeconds(0.5)
                        .build();
                TrajectorySequence To_BackBoard = drive.trajectorySequenceBuilder(Middle.end())
                        .lineToLinearHeading(new Pose2d(26, 85, Math.toRadians(260)))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, 40, 15.30))
                        .back(3)
                        .build();
                TrajectorySequence Score1 = drive.trajectorySequenceBuilder(To_BackBoard.end())
                        .addDisplacementMarker(() -> {
                            linearSlide_left.setPower(0.5);
                            linearSlide_right.setPower(0.5);
                        })
                        .waitSeconds(2.2)
                        .build();
                TrajectorySequence Score2 = drive.trajectorySequenceBuilder(Score1.end())
                        .addDisplacementMarker(() -> {
                            Outtake_left.setPosition(0.5);
                            Outtake_right.setPosition(0.52);
                        })
                        .waitSeconds(1)
                        .build();
                TrajectorySequence Score3 = drive.trajectorySequenceBuilder(Score2.end())
                        .addDisplacementMarker(() -> {
                            OutDoor.setPosition(0.4);
                        })
                        .waitSeconds(1)
                        .build();
                TrajectorySequence Park = drive.trajectorySequenceBuilder(Score3.end())
                        .addDisplacementMarker(() -> {
                            linearSlide_left.setPower(-0.3);
                            linearSlide_right.setPower(-0.3);
                        })
                        .addDisplacementMarker(() -> {
                            OutDoor.setPosition(0.1);
                        })
                        .addDisplacementMarker(() -> {
                            Outtake_left.setPosition(0.75);
                            Outtake_right.setPosition(0.28);
                        })
                        .forward(15)
                        .lineToLinearHeading(new Pose2d(54, 82, Math.toRadians(260)))
                        .back(15)
                        .build();
                drive.followTrajectorySequence(To_Marker);
                drive.followTrajectorySequence(Middle);
                drive.followTrajectorySequence(To_BackBoard);
                drive.followTrajectorySequence(Score1);
                drive.followTrajectorySequence(Score2);
                drive.followTrajectorySequence(Score3);
                drive.followTrajectorySequence(Park);
                break;
            }
        }
    }
}
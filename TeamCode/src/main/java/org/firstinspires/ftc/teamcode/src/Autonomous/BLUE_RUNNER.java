package org.firstinspires.ftc.teamcode.src.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.src.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.src.vision.PipeLine_BLUE;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name = "BLUE_RUNNER", group = "COMPETITION")

public class BLUE_RUNNER extends LinearOpMode {
    private OpenCvCamera webcam;


    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 180; // height of wanted camera resolution

    private double CrLowerUpdate = 160;
    private double CbLowerUpdate = 100;
    private double CrUpperUpdate = 255;
    private double CbUpperUpdate = 255;

    public static double borderLeftX = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY = 0.0;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY = 0.0;   //fraction of pixels from the bottom of the cam to skip

    private double lowerruntime = 0;
    private double upperruntime = 0;

    // Pink Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 60, 150);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);

    static final Rect Left = new Rect(
            new Point(60, 35), new Point(120, 75));
    static final Rect Right = new Rect(
            new Point(140, 35), new Point(200, 75));


    // Yellow Range
//    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 100.0, 0.0);
//    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 170.0, 120.0);

    @Override
    public void runOpMode() {
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
            if (myPipeline.getRectArea() > 1100) {
                if (myPipeline.getRectMidpointX() < 110) {
                    AUTONOMOUS_C();
                } else if (myPipeline.getRectMidpointX() > 150 && myPipeline.getRectMidpointX() < 200) {
                    AUTONOMOUS_B();
                } else if (myPipeline.getRectMidpointX() > 200) {
                    AUTONOMOUS_A();
                }
                telemetry.addData("RectArea: ", myPipeline.getRectArea());
                telemetry.addData("X", myPipeline.getRectX());
                telemetry.addData("MidPoint", myPipeline.getRectMidpointX());
                telemetry.addData("MidPoint", myPipeline.getRectHeight());
                telemetry.update();
            } else telemetry.addLine("NOTHING FOUND");
            telemetry.update();
        }


        while (opModeIsActive()) {
            myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
            if (myPipeline.error) {
                telemetry.addData("Exception: ", myPipeline.debug);
            }

            telemetry.addData("RectArea: ", myPipeline.getRectArea());
            telemetry.addData("X", myPipeline.getRectX());
            telemetry.addData("MidPoint", myPipeline.getRectMidpointX());
            telemetry.addData("MidPoint", myPipeline.getRectHeight());

            telemetry.update();

            if (myPipeline.getRectArea() > 1100) {
                if (myPipeline.getRectMidpointX() < 110) {
                    TrajectorySequence To_Junction = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                            .lineToConstantHeading(new Vector2d(62, 0))
                            .back(10)
                            .build();
                    drive.followTrajectorySequence(To_Junction);
                } else if (myPipeline.getRectMidpointX() > 150 && myPipeline.getRectMidpointX() < 200) {
                    AUTONOMOUS_B();
                } else if (myPipeline.getRectMidpointX() > 200) {
                    AUTONOMOUS_A();
                }
            }
        }
    }

    public void AUTONOMOUS_A() {
        telemetry.addLine("RIGHT");
        telemetry.update();
    }

    public void AUTONOMOUS_B() {
        telemetry.addLine("CENTER");
        telemetry.update();
    }

    public void AUTONOMOUS_C() {
        telemetry.addLine("LEFT");
        telemetry.update();
    }
}
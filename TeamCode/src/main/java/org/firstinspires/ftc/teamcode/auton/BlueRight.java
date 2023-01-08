package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.subsystems.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(group = "drive")

public class BlueRight extends LinearOpMode {

    Robot robot;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1, 2, 3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot = new Robot(hardwareMap, true);
        Pose2d startPose = new Pose2d(in(92), in(160), rad(90));
        drive.setPoseEstimate(startPose);
        robot.claw.setPositionClaw(0.8);
        robot.autoInit(true);

        TrajectorySequence preloadTrajectory = drive.trajectorySequenceBuilder(startPose)
                .back(50)
                .addDisplacementMarker(1, ()-> {
                    robot.autoHigh(true);
                    robot.guide.setGuideDown();
                })
                .setReversed(true)
                .addDisplacementMarker(32, ()-> {
                    robot.slides.runToPreset(Levels.HIGH);
                })
                .splineTo(new Vector2d(30.5,7), 179.8)
                .addDisplacementMarker(54,()->{
                    robot.slides.runToPosition(-320);
                })
                .addTemporalMarker(2.3, ()->{
                    robot.claw.setClawOpen();
                })
                .build();

        TrajectorySequence poleToStackTrajectory1 = drive.trajectorySequenceBuilder(preloadTrajectory.end())
                .addTemporalMarker(0, ()->{
                    robot.autoLow(true);
                })
                .waitSeconds(1)
                .setReversed(false)
                .splineTo(new Vector2d(56,8.5), 0)
                .addDisplacementMarker(25.5, ()->{
                    robot.claw.setClawClose();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(2.8, ()->{
                    robot.slides.runToPreset(Levels.HIGH);
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence stackToHighTrajectory1 = drive.trajectorySequenceBuilder(poleToStackTrajectory1.end())
                .setReversed(true)
                .splineTo(new Vector2d(32,7), 179.8)
                .addTemporalMarker(1, ()->{
                    robot.autoHigh(true);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(1.5, ()->{
                    robot.slides.runToPosition(-280);
                })
                .waitSeconds(0.5)
                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        robot.slides.launchAsThread(telemetry);
        robot.guide.setGuideDown();
        drive.followTrajectorySequence(preloadTrajectory);
        drive.followTrajectorySequence(poleToStackTrajectory1);
        drive.followTrajectorySequence(stackToHighTrajectory1);
        robot.claw.setClawOpen();
//        drive.followTrajectorySequence(trajectory);
//
//        TrajectorySequence parkTrajectory = null;
//        /* Actually do something useful */
//        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
//            // insert trajectory code
//            parkTrajectory = robot.drive.trajectorySequenceBuilder(trajectory.end())
//                    .setReversed(false)
//                    .splineTo(new Vector2d(35,13), 0)
//                    .forward(22)
//                    .build();
//        } else if (tagOfInterest.id == MIDDLE) {
//            // insert trajectory code
//            parkTrajectory = robot.drive.trajectorySequenceBuilder(trajectory.end())
//                    .setReversed(false)
//                    .splineTo(new Vector2d(35,13), 0)
//                    .build();
//        } else if (tagOfInterest.id == RIGHT) {
//            // insert trajectory code
//            parkTrajectory = robot.drive.trajectorySequenceBuilder(trajectory.end())
//                    .setReversed(false)
//                    .splineTo(new Vector2d(35,13), 0)
//                    .back(22)
//                    .build();
//        }

//        robot.drive.followTrajectorySequence(parkTrajectory);

        robot.slides.destroyThreads(telemetry);
        while (!isStopRequested() && opModeIsActive()) ;
    }

    public static double rad(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double in(double centimeters) {
        return centimeters * 0.3837008;
    }

}

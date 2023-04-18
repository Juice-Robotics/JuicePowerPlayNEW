package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
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

public class LeftStackCONSISTENT extends LinearOpMode {

    Robot robot;

    // Tag ID 1, 2, 3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    double WAIT_1 = 0.5;
    double WAIT_2 = 0.5;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot = new Robot(hardwareMap, true);
        Pose2d startPose = new Pose2d(in(92), in(165), rad(90));
        drive.setPoseEstimate(startPose);
        PhotonCore.enable();
        robot.autoInitTrue(true);
        robot.claw.setClawClose();
        robot.claw.setYRotation(142);

        TrajectorySequence preloadTrajectory = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .back(48)
                .splineTo(new Vector2d(30,5.5), Math.toRadians(226))
                .addTemporalMarker(1.3, ()->{
                    robot.highPreset(true);
                })
                .addTemporalMarker(2.4, ()->{
                    robot.autoDeposit(true);
                })
                .addTemporalMarker(2.6, ()->{
                    robot.slides.runToPosition(-270);
                })
                .waitSeconds(WAIT_1)
                .build();

        TrajectorySequence poleToStackTrajectory1 = drive.trajectorySequenceBuilder(preloadTrajectory.end())
                .addTemporalMarker(0.5, ()->{
                    robot.autoLow(true);
                })
                .setReversed(false)
                .splineTo(new Vector2d(58,10.5), Math.toRadians(0))
                .addTemporalMarker(1.2, ()->{
                    robot.claw.setClawClose();
                })
                .addTemporalMarker(1.6, ()->{
                    robot.autoInit(true);
                })
                .waitSeconds(WAIT_1)
                .build();

        TrajectorySequence stackToHighTrajectory1 = drive.trajectorySequenceBuilder(poleToStackTrajectory1.end())
                .setReversed(true)
                .splineTo(new Vector2d(32,5), Math.toRadians(226))
                .addTemporalMarker(0.4, ()->{
                    robot.autoHigh(true);
                })
                .addTemporalMarker(1.6, ()->{
                    robot.autoDeposit(true);
                })
                .addTemporalMarker(1.9, ()->{
                    robot.slides.runToPosition(-210);
                })
                .waitSeconds(WAIT_2)
                .build();

        TrajectorySequence poleToStackTrajectory2 = drive.trajectorySequenceBuilder(stackToHighTrajectory1.end())
                .addTemporalMarker(0.5, ()->{
                    robot.autoLow(true);
                })
                .setReversed(false)
                .splineTo(new Vector2d(58,10.5), Math.toRadians(0))
                .addTemporalMarker(1.2, ()->{
                    robot.claw.setClawClose();
                })
                .addTemporalMarker(1.6, ()->{
                    robot.autoInit(true);
                })
                .waitSeconds(WAIT_1)
                .build();

        TrajectorySequence stackToHighTrajectory2 = drive.trajectorySequenceBuilder(poleToStackTrajectory2.end())
                .setReversed(true)
                .splineTo(new Vector2d(32,5), Math.toRadians(226))
                .addTemporalMarker(0.4, ()->{
                    robot.autoHigh(true);
                })
                .addTemporalMarker(1.6, ()->{
                    robot.autoDeposit(true);
                })
                .addTemporalMarker(1.9, ()->{
                    robot.slides.runToPosition(-160);
                })
                .waitSeconds(WAIT_2)
                .build();

        TrajectorySequence poleToStackTrajectory3 = drive.trajectorySequenceBuilder(stackToHighTrajectory2.end())
                .addTemporalMarker(0.5, ()->{
                    robot.autoLow(true);
                })
                .setReversed(false)
                .splineTo(new Vector2d(58,10.5), Math.toRadians(0))
                .addTemporalMarker(1.2, ()->{
                    robot.claw.setClawClose();
                })
                .addTemporalMarker(1.6, ()->{
                    robot.autoInit(true);
                })
                .waitSeconds(WAIT_1)
                .build();

        TrajectorySequence stackToHighTrajectory3 = drive.trajectorySequenceBuilder(poleToStackTrajectory3.end())
                .setReversed(true)
                .splineTo(new Vector2d(32,5), Math.toRadians(226))
                .addTemporalMarker(0.4, ()->{
                    robot.autoHigh(true);
                })
                .addTemporalMarker(1.6, ()->{
                    robot.autoDeposit(true);
                })
                .addTemporalMarker(1.9, ()->{
                    robot.slides.runToPosition(-55);
                })
                .waitSeconds(WAIT_2)
                .build();

        TrajectorySequence poleToStackTrajectory4 = drive.trajectorySequenceBuilder(stackToHighTrajectory3.end())
                .addTemporalMarker(0.5, ()->{
                    robot.autoLow(true);
                })
                .setReversed(false)
                .splineTo(new Vector2d(58,10.5), Math.toRadians(0))
                .addTemporalMarker(1.2, ()->{
                    robot.claw.setClawClose();
                })
                .addTemporalMarker(1.6, ()->{
                    robot.autoInit(true);
                })
                .waitSeconds(WAIT_1)
                .build();

        TrajectorySequence stackToHighTrajectory4 = drive.trajectorySequenceBuilder(poleToStackTrajectory4.end())
                .setReversed(true)
                .splineTo(new Vector2d(32,5), Math.toRadians(226))
                .addTemporalMarker(0.4, ()->{
                    robot.autoHigh(true);
                })
                .addTemporalMarker(1.6, ()->{
                    robot.autoDeposit(true);
                })
                .addTemporalMarker(1.9, ()->{
                    robot.slides.runToPosition(-15);
                })
                .waitSeconds(WAIT_2)
                .build();

        TrajectorySequence poleToStackTrajectory5 = drive.trajectorySequenceBuilder(stackToHighTrajectory4.end())
                .addTemporalMarker(0.5, ()->{
                    robot.autoLow(true);
                })
                .setReversed(false)
                .splineTo(new Vector2d(58,11), Math.toRadians(0))
                .addTemporalMarker(1.2, ()->{
                    robot.claw.setClawClose();
                })
                .addTemporalMarker(1.6, ()->{
                    robot.autoInit(true);
                })
                .waitSeconds(WAIT_1)
                .build();

        TrajectorySequence stackToHighTrajectory5 = drive.trajectorySequenceBuilder(poleToStackTrajectory5.end())
                .setReversed(true)
                .splineTo(new Vector2d(32,5), Math.toRadians(226))
                .addTemporalMarker(0.4, ()->{
                    robot.autoHigh(true);
                })
                .addTemporalMarker(1.4, ()->{
                    robot.autoDeposit(true);
                })
                .addTemporalMarker(1.5, ()->{
                    robot.slides.runToPosition(0);
                })
                .addTemporalMarker(2.1, ()->{
                    robot.autoLow(true);
                })
                .waitSeconds(WAIT_2)
                .build();

       robot.cv.observeSleeve();
        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = robot.cv.getSleevePark();

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
                    telemetry.addData("Location: ", tagOfInterest.id);
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
        drive.followTrajectorySequence(preloadTrajectory);
        drive.followTrajectorySequence(poleToStackTrajectory1);
        drive.followTrajectorySequence(stackToHighTrajectory1);
        drive.followTrajectorySequence(poleToStackTrajectory2);
        drive.followTrajectorySequence(stackToHighTrajectory2);
        drive.followTrajectorySequence(poleToStackTrajectory3);
        drive.followTrajectorySequence(stackToHighTrajectory3);
        drive.followTrajectorySequence(poleToStackTrajectory4);
        drive.followTrajectorySequence(stackToHighTrajectory4);
        drive.followTrajectorySequence(poleToStackTrajectory5);
        drive.followTrajectorySequence(stackToHighTrajectory5);

        TrajectorySequence parkTrajectory = null;
        /* Actually do something useful */
        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            // insert trajectory code
            parkTrajectory = robot.drive.trajectorySequenceBuilder(stackToHighTrajectory5.end())
                    .waitSeconds(0.2)
                    .setReversed(false)
                    .splineTo(new Vector2d(58,11), Math.toRadians(0))
                    .addTemporalMarker(0.8, ()->{
                        robot.autoInit(true);
                    })
                    .build();
        } else if (tagOfInterest.id == MIDDLE) {
            // insert trajectory code
            parkTrajectory = robot.drive.trajectorySequenceBuilder(stackToHighTrajectory5.end())
                    .waitSeconds(0.2)
                    .setReversed(false)
                    .splineTo(new Vector2d(34,38), Math.toRadians(90))
                    .addTemporalMarker(0.8, ()->{
                        robot.autoInit(true);
                    })
                    .build();
        } else if (tagOfInterest.id == RIGHT) {
            // insert trajectory code
            parkTrajectory = robot.drive.trajectorySequenceBuilder(stackToHighTrajectory5.end())
                    .waitSeconds(0.2)
                    .setReversed(false)
                    .splineTo(new Vector2d(35,14), Math.toRadians(90))
                    .strafeLeft(24)
                    .forward(20) //could be extra
                    .addTemporalMarker(0.8, ()->{
                        robot.autoInit(true);
                    })
                    .build();
        }

        drive.followTrajectorySequence(parkTrajectory);

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
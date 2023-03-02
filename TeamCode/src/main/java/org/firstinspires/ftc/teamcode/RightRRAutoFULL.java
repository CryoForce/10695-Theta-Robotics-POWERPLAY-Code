package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name="RightRRAutoFULL", group="Auton")
public class RightRRAutoFULL extends LinearOpMode {



    LinearOpMode op = this;
    /* Declare OpMode members. */
    ThetaHardware robot = new ThetaHardware(this);   // Use a Theta's hardware

    Constants constants = new Constants();
    private ElapsedTime runtime = new ElapsedTime();
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;


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

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;


    int aprilTPos = 0;

    int cyclenum = 0;
    int y = 0;
    int c = 0;
    int h = 0;
    int e = 0;

    enum Stage {pole1, scorepole1, score, topole, park, end, score1,toconestack, scorepole,
        score2, scorepole2, toconestack2, topole2, score3, scorepole3, toconestack3, topole3 }
    Stage stage = Stage.pole1;
    ElapsedTime scoreTime = new ElapsedTime();
    ElapsedTime dropTime = new ElapsedTime();
    ElapsedTime liftTime = new ElapsedTime();
    ElapsedTime clawTime = new ElapsedTime();



    int liftPos = 0;
    static int target = 0;
    private final double ticks_in_degrees = 384.5 / 360;
    PIDController controller;

    public static double p = 0.016, i = 0, d = 0.001;
    public static double f = 0.075;

    Constants cons = new Constants();



    //-----Trajectories-----//
    Trajectory scorepole1p;
    Trajectory topark2;


    @Override
    public void runOpMode() throws InterruptedException {

        controller = new PIDController(p, i, d);



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "thetaWebcam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });



        scorepole1p = drive.trajectoryBuilder(topark2.end())
                .lineToLinearHeading(new Pose2d(51, -0.5, Math.toRadians(45)))
                .build();


        topark2 = drive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(44, -2),0)
                .addDisplacementMarker(() ->
                    drive.followTrajectoryAsync(scorepole1p)
                )
                .build();




        //----------1st Cycle----------//



        Trajectory lineconestack1 = drive.trajectoryBuilder(scorepole1p.end())
                .lineToLinearHeading(new Pose2d(49.75, 0, Math.toRadians(95)))
                .build();

        Trajectory toconestack1 = drive.trajectoryBuilder(lineconestack1.end())
                .lineToLinearHeading(new Pose2d(49.75, -29, Math.toRadians(95)))
                .build();

        Trajectory topoles = drive.trajectoryBuilder(toconestack1.end())
                .lineToLinearHeading(new Pose2d(51, -1, Math.toRadians(95)))
                .build();

        Trajectory scorepole = drive.trajectoryBuilder(topoles.end())
                .lineToLinearHeading(new Pose2d(51, -3.5, Math.toRadians(45)))
                .build();


        //----------2nd Cycle----------//

        Trajectory lineconestack2 = drive.trajectoryBuilder(scorepole.end())
                .lineToLinearHeading(new Pose2d(49.75, 0, Math.toRadians(95)))
                .build();

        Trajectory toconestack2 = drive.trajectoryBuilder(lineconestack2.end())
                .lineToLinearHeading(new Pose2d(49.75, -29, Math.toRadians(95)))
                .build();

        Trajectory topoles2 = drive.trajectoryBuilder(toconestack2.end())
                .lineToLinearHeading(new Pose2d(51, -1, Math.toRadians(95)))
                .build();

        Trajectory scorepole2 = drive.trajectoryBuilder(topoles2.end())
                .lineToLinearHeading(new Pose2d(51, -3, Math.toRadians(45)))
                .build();



        //----------3rd Cycle----------//

        Trajectory lineconestack3 = drive.trajectoryBuilder(scorepole2.end())
                .lineToLinearHeading(new Pose2d(49.75, 0, Math.toRadians(95)))
                .build();

        Trajectory toconestack3 = drive.trajectoryBuilder(lineconestack3.end())
                .lineToLinearHeading(new Pose2d(49.75, -29, Math.toRadians(95)))
                .build();

        Trajectory topoles3 = drive.trajectoryBuilder(toconestack3.end())
                .lineToLinearHeading(new Pose2d(51, -1, Math.toRadians(95)))
                .build();

        Trajectory scorepole3 = drive.trajectoryBuilder(topoles3.end())
                .lineToLinearHeading(new Pose2d(51, -3.5, Math.toRadians(45)))
                .build();




        Trajectory park2 = drive.trajectoryBuilder(scorepole3.end())
                .lineToLinearHeading( new Pose2d(51, -2, Math.toRadians(0)))
                .build();

        Trajectory park1 = drive.trajectoryBuilder(park2.end())
                .lineToLinearHeading( new Pose2d(49.75, 23, Math.toRadians(90)))
                .build();

        Trajectory park3 = drive.trajectoryBuilder(park2.end())
                .lineToLinearHeading( new Pose2d(49.75, -28, Math.toRadians(90)))
                .build();







        robot.RRinit(hardwareMap, telemetry);

//         Send telemetry message to signify robot waiting;



        robot.rightV4b.setPosition(0.88);
        robot.leftV4b.setPosition(0.88);
        liftPos = 0;


        robot.rightClaw.setPosition(0.2);
        robot.leftClaw.setPosition(0.47);

        while (!opModeIsActive())
        {
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.

            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();


            // If there's been a new frame...
            if(detections != null)
            {
                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                // If we don't see any tags
                if(detections.size() == 0)
                {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else
                {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    for(AprilTagDetection detection : detections)
                    {
                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                        aprilTPos = detection.id;

                    }
                }

                telemetry.update();
            }

            sleep(20);

        }



        waitForStart();


        while (opModeIsActive() && !isStopRequested()) {

            drive.update();
            controller.setPID(p, i, d);
            int armPos = robot.liftOther.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

            double liftPower = pid + ff;

            robot.liftOther.setPower(liftPower);
            robot.liftMain.setPower(liftPower);

            if (liftPos == 0) {
                target = 25;

            } else if (liftPos == 1) {
                target = 925;


            } else if (liftPos == 2) {
                target = 1700;

            }

            switch (stage) {
                case pole1:
                    robot.v4bUp();

                    drive.followTrajectoryAsync(topark2);

                    stage = Stage.scorepole1;
                    break;

                case scorepole1:


                    liftPos = 2;

                    if(!drive.isBusy()) {



                        dropTime.reset();
                        liftTime.reset();
                        scoreTime.reset();


                        stage = Stage.score1;

                    }

                case score1:
                    if(!drive.isBusy()) {

                        robot.v4bSH();

                        if(scoreTime.milliseconds() > 1600) {

                            robot.openClaw();

                        }
                        if (dropTime.milliseconds() > 1850) {

                            robot.rightV4b.setPosition(cons.topCone);
                            robot.leftV4b.setPosition(cons.topCone);

                        }
                        if (liftTime.milliseconds() > 1900) {
                            liftPos = 0;
                            stage = Stage.toconestack;
                        }
                    }


                    break;

                case toconestack:
                    liftPos = 0;
                    if(y == 0) {
                        if (!drive.isBusy()) {
                            drive.followTrajectoryAsync(lineconestack1);
                            y++;
                        }

                    }
                    if(y == 1){
                        if(!drive.isBusy()) {
                            drive.followTrajectoryAsync(toconestack1);

                            y++;
                        }
                    }
                    if(y == 2){
                        if(!drive.isBusy()) {
                            robot.closeClaw();
                            stage = Stage.topole;
                        }
                    }
                    break;

                case topole:

                    if(h == 0){
                        liftTime.reset();
                        scoreTime.reset();
                        dropTime.reset();
                        clawTime.reset();

                        h++;
                    }
                    if(h == 1) {
                        if(!drive.isBusy()) {
                            if (clawTime.milliseconds() > 200) {
                                robot.v4bUp();
                                h++;
                            }
                        }
                    }
                    if(h == 2){
                        if(!drive.isBusy()){
                                liftPos = 2;
                                drive.followTrajectoryAsync(topoles);
                                stage = Stage.scorepole;

                        }
                    }
                    break;


                case scorepole:

                    if(!drive.isBusy()) {

                        drive.followTrajectoryAsync(scorepole);

                        stage = Stage.score;

                    }
                    break;

                case score:
                    if(!drive.isBusy()) {
                        if (e == 0) {
                            liftTime.reset();
                            scoreTime.reset();
                            dropTime.reset();
                            clawTime.reset();

                            e++;
                        }
                    }
                    if (e == 1) {
                        if (!drive.isBusy()) {

                            robot.v4bSH();

                            if (scoreTime.milliseconds() > 400) {

                                robot.openClaw();
                                y = 0;
                                h = 0;

                            }
                            if (dropTime.milliseconds() > 550) {

                                robot.rightV4b.setPosition(cons.secondCone);
                                robot.leftV4b.setPosition(cons.secondCone);

                            }
                            if (liftTime.milliseconds() > 575) {
                                liftPos = 0;

                                stage = Stage.toconestack2;

                            }
                        }
                    }


                    break;




                case toconestack2:
                    liftPos = 0;
                    if(y == 0) {
                        if (!drive.isBusy()) {
                            drive.followTrajectoryAsync(lineconestack2);
                            e = 0;
                            y++;
                        }

                    }
                    if(y == 1){
                        if(!drive.isBusy()) {
                            drive.followTrajectoryAsync(toconestack2);

                            y++;
                        }
                    }
                    if(y == 2){
                        if(!drive.isBusy()) {
                            robot.closeClaw();
                            stage = Stage.topole2;
                        }
                    }
                    break;

                case topole2:

                    if(h == 0){
                        liftTime.reset();
                        scoreTime.reset();
                        dropTime.reset();
                        clawTime.reset();

                        h++;
                    }
                    if(h == 1) {
                        if(!drive.isBusy()) {
                            if (clawTime.milliseconds() > 200) {
                                robot.v4bUp();
                                h++;
                            }
                        }
                    }
                    if(h == 2){
                        if(!drive.isBusy()){
                            liftPos = 2;
                            drive.followTrajectoryAsync(topoles2);
                            stage = Stage.scorepole2;

                        }
                    }
                    break;


                case scorepole2:

                    if(!drive.isBusy()) {

                        drive.followTrajectoryAsync(scorepole2);

                        stage = Stage.score2;

                    }
                    break;

                case score2:
                    if(!drive.isBusy()) {
                        if (e == 0) {
                            liftTime.reset();
                            scoreTime.reset();
                            dropTime.reset();
                            clawTime.reset();

                            e++;
                        }
                    }
                    if (e == 1) {
                        if (!drive.isBusy()) {

                            robot.v4bSH();

                            if (scoreTime.milliseconds() > 400) {

                                robot.openClaw();

                            }
                            if (dropTime.milliseconds() > 550) {

                                robot.rightV4b.setPosition(cons.secondCone);
                                robot.leftV4b.setPosition(cons.secondCone);

                            }
                            if (liftTime.milliseconds() > 575) {
                                liftPos = 0;
                                h = 0;
                                y = 0;

                                stage = Stage.toconestack3;

                            }
                        }
                    }


                    break;


                case toconestack3:
                    liftPos = 0;
                    if(y == 0) {
                        if (!drive.isBusy()) {
                            drive.followTrajectoryAsync(lineconestack3);
                            e = 0;
                            y++;
                        }

                    }
                    if(y == 1){
                        if(!drive.isBusy()) {
                            drive.followTrajectoryAsync(toconestack3);

                            y++;
                        }
                    }
                    if(y == 2){
                        if(!drive.isBusy()) {
                            robot.closeClaw();
                            stage = Stage.topole3;
                        }
                    }
                    break;

                case topole3:

                    if(h == 0){
                        liftTime.reset();
                        scoreTime.reset();
                        dropTime.reset();
                        clawTime.reset();

                        h++;
                    }
                    if(h == 1) {
                        if(!drive.isBusy()) {
                            if (clawTime.milliseconds() > 200) {
                                robot.v4bUp();
                                h++;
                            }
                        }
                    }
                    if(h == 2){
                        if(!drive.isBusy()){
                            liftPos = 2;
                            drive.followTrajectoryAsync(topoles3);
                            stage = Stage.scorepole3;
                        }
                    }
                    break;


                case scorepole3:

                    if(!drive.isBusy()) {

                        drive.followTrajectoryAsync(scorepole3);

                        stage = Stage.score3;

                    }
                    break;

                case score3:
                    if(!drive.isBusy()) {
                        if (e == 0) {
                            liftTime.reset();
                            scoreTime.reset();
                            dropTime.reset();
                            clawTime.reset();

                            e++;
                        }
                    }
                    if (e == 1) {
                        if (!drive.isBusy()) {

                            robot.v4bSH();

                            if (scoreTime.milliseconds() > 400) {

                                robot.openClaw();

                            }
                            if (dropTime.milliseconds() > 550) {

                                robot.rightV4b.setPosition(cons.fourthCone);
                                robot.leftV4b.setPosition(cons.fourthCone);

                            }
                            if (liftTime.milliseconds() > 575) {
                                liftPos = 0;

                                stage = Stage.park;

                            }
                        }
                    }


                    break;






                case park:
                    if (aprilTPos == 1) {
                        if(c == 0) {
                            if (!drive.isBusy()) {
                                liftPos = 0;
                                drive.followTrajectoryAsync(park2);
                                c++;
                            }
                        }
                        if(c == 1) {
                            if (!drive.isBusy()) {
                                liftPos = 0;
                                drive.followTrajectoryAsync(park1);
                                stage = Stage.end;
                            }
                        }
                    }
                    if (aprilTPos == 2) {

                        if(!drive.isBusy()) {
                            liftPos = 0;
                            drive.followTrajectoryAsync(park2);
                            stage = Stage.end;
                        }

                    }
                    if (aprilTPos == 3) {
                        if(c == 0) {
                            if (!drive.isBusy()) {
                                liftPos = 0;
                                drive.followTrajectoryAsync(park2);
                                c++;
                            }
                        }
                        if(c == 1) {
                            if (!drive.isBusy()) {
                                liftPos = 0;
                                drive.followTrajectoryAsync(park3);
                                stage = Stage.end;
                            }
                        }
                    }
                    break;

                case end:

                    break;

            }
        }



    }
}

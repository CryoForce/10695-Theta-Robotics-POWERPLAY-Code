/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


//import org.firstinspires.ftc.robotcontroller.external.samples.HardwareTheta;


@Autonomous(name="Drive Auto Right", group="Auton")
//@Disabled
public class DriveAutoRight extends LinearOpMode {

    LinearOpMode op = this;
    /* Declare OpMode members. */
    ThetaHardware robot = new ThetaHardware(this);   // Use a Theta's hardware


    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;


    double scoringPos;
    int v4bPos;
    boolean v4bDown;
    int lowPole;
    static int target = 0;
    private final double ticks_in_degrees = 384.5 / 360;
    int liftPos = 0;

    PIDController controller;

    public static double p = 0.018, i = 0, d = 0.001;
    public static double f = 0.075;


    ElapsedTime clawTime = new ElapsedTime();
    ElapsedTime v4bTime = new ElapsedTime();
    ElapsedTime liftTime = new ElapsedTime();
    ElapsedTime clawOpenTime = new ElapsedTime();
    ElapsedTime liftWaitTime = new ElapsedTime();


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

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;


    int aprilTPos = 0;


    @Override
    public void runOpMode() {


//        controller = new PIDController(p, i, d);


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

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, telemetry);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.initMotors();

        telemetry.setMsTransmissionInterval(50);

        robot.rightV4b.setPosition(0.85);
        robot.leftV4b.setPosition(0.85);


        robot.rightClaw.setPosition(0.2);
        robot.leftClaw.setPosition(0.47);
        robot.leftH.setPosition(0.64);
        robot.rightH.setPosition(0.64);

        clawTime.reset();
        v4bTime.reset();
        liftTime.reset();
        clawOpenTime.reset();
        liftWaitTime.reset();


        while (opModeInInit()) {

            // Call
            //
            //
            //
            //
            //
            //
            //
            //
            //
            // ing getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.

            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();


            // If there's been a new frame...
            if (detections != null) {
                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                // If we don't see any tags
                if (detections.size() == 0) {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    for (AprilTagDetection detection : detections) {
                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                        aprilTPos = detection.id;

                    }
                }

                telemetry.update();
            }

            sleep(20);

        }


        while (opModeIsActive()) {

            controller.setPID(p, i, d);
            int armPos = robot.liftOther.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

            double liftPower = pid + ff;

            robot.liftOther.setPower(liftPower);
            robot.liftMain.setPower(liftPower);


            if (liftPos == 0) {
              target = 25;
            }else if (liftPos == 2) {
                target = 1690;
            }


            //
            //
            //


            if (scoringPos == -1) {
                clawTime.reset();
                v4bTime.reset();
                liftTime.reset();
                clawOpenTime.reset();
                liftWaitTime.reset();


                v4bDown = false;

                v4bPos = 1;
                lowPole = 3;


            } else if (scoringPos == 0) {

                //Open Claw
                robot.rightClaw.setPosition(0.34);
                robot.leftClaw.setPosition(0.34);

                if (clawTime.milliseconds() > 600) {
                    //Pull Horizontal Extension In
                    robot.leftH.setPosition(0.61);
                    robot.rightH.setPosition(0.64);
                }

                if (liftTime.milliseconds() > 900) {
                    //Lower Lift

                    liftPos = 0;
                    v4bPos = 0;
                    clawOpenTime.reset();
                    v4bTime.reset();

                }


                if (v4bPos == 0) {


                    if (v4bTime.milliseconds() > 500) {
                        robot.rightV4b.setPosition(0.866);
                        robot.leftV4b.setPosition(0.866);
                    }

                    if (clawOpenTime.milliseconds() > 900) {
                        robot.rightClaw.setPosition(0.37);
                        robot.leftClaw.setPosition(0.31);

                    }


                    scoringPos = -1;
                }


            } else if (scoringPos == 1) {
                lowPole = 1;

                //Close Claw
                robot.rightClaw.setPosition(0.20);
                robot.leftClaw.setPosition(0.47);

                if (clawTime.milliseconds() > 250) {

                    //Raise Virtual 4 Bar
                    robot.rightV4b.setPosition(0.41);
                    robot.leftV4b.setPosition(0.41);
                    v4bPos = 3;

                }
                if (v4bPos == 3) {
                    if (liftWaitTime.milliseconds() > 550) {
                        //Raise Lift
                        liftPos = 2;
                    }
                }
                if (armPos > 150) {
                    //Extend Horizontal Extension
                    robot.rightH.setPosition(0.36);
                    robot.leftH.setPosition(0.36);
                    v4bPos = 0;

                }
                if (v4bPos == 0) {
                    scoringPos = -1;
                }

            } else if (scoringPos == 2) {
                lowPole = 1;

                //Close Claw
                robot.rightClaw.setPosition(0.20);
                robot.leftClaw.setPosition(0.47);

                if (clawTime.milliseconds() > 250) {

                    //Raise Virtual 4 Bar
                    robot.rightV4b.setPosition(0.41);
                    robot.leftV4b.setPosition(0.41);
                    v4bPos = 3;

                }
                if (v4bPos == 3) {
                    if (liftWaitTime.milliseconds() > 550) {
                        //Raise Lift
                        liftPos = 2;
                    }
                }
                if (armPos > 150) {
                    //Extend Horizontal Extension
                    robot.rightH.setPosition(0.36);
                    robot.leftH.setPosition(0.36);
                    v4bPos = 0;

                }
                if (v4bPos == 0) {
                    scoringPos = -1;
                }


            }


            if (aprilTPos == 1) {


                robot.strafeForCounts(75, -.5, -.5, 3000);

                //drive to pole
                robot.driveForCounts(690, .5, .5, 2000);

                robot.thetaWait(0.05);

                robot.driveForCounts(440, .5, -.5, 2000);

                robot.thetaWait(0.05);

                robot.driveForCounts(25, .5, .5, 2000);


                robot.rightV4b.setPosition(0.38);
                robot.leftV4b.setPosition(0.38);

                robot.thetaWait(0.05);

                robot.rightV4b.setPosition(0.38);
                robot.leftV4b.setPosition(0.38);

                robot.thetaWait(1.5);

                robot.rightClaw.setPosition(0.34);
                robot.leftClaw.setPosition(0.34);

                robot.thetaWait(0.5);

                robot.rightV4b.setPosition(0.876);
                robot.leftV4b.setPosition(0.876);

                robot.driveForCounts(25, -.5, -.5, 2000);


                robot.driveForCounts(410, -.5, .5, 2000);

                robot.strafeForCounts(875, .5, .5, 3000);


            }
            if (aprilTPos == 2) {

                robot.strafeForCounts(75, -.5, -.5, 3000);

                //drive to pole
                robot.driveForCounts(690, .5, .5, 2000);

                robot.thetaWait(0.05);

                robot.driveForCounts(440, .5, -.5, 2000);

                robot.thetaWait(0.05);


                robot.driveForCounts(25, .5, .5, 2000);

                robot.rightV4b.setPosition(0.38);
                robot.leftV4b.setPosition(0.38);

                robot.thetaWait(0.05);

                robot.rightV4b.setPosition(0.38);
                robot.leftV4b.setPosition(0.38);

                robot.thetaWait(1.5);

                robot.rightClaw.setPosition(0.34);
                robot.leftClaw.setPosition(0.34);

                robot.thetaWait(0.5);

                robot.rightV4b.setPosition(0.876);
                robot.leftV4b.setPosition(0.876);

                robot.driveForCounts(25, -.5, -.5, 2000);


                robot.driveForCounts(410, -.5, .5, 2000);


            }
            if (aprilTPos == 3) {

                robot.strafeForCounts(75, -.5, -.5, 3000);

                //drive to pole
                robot.driveForCounts(700, .5, .5, 2000);

                robot.thetaWait(0.05);

                robot.driveForCounts(440, .5, -.5, 2000);

                robot.thetaWait(0.05);

                robot.driveForCounts(25, .5, .5, 2000);


                robot.rightV4b.setPosition(0.38);
                robot.leftV4b.setPosition(0.38);

                robot.thetaWait(0.05);

                robot.rightV4b.setPosition(0.38);
                robot.leftV4b.setPosition(0.38);

                robot.thetaWait(1.5);

                robot.rightClaw.setPosition(0.34);
                robot.leftClaw.setPosition(0.34);

                robot.thetaWait(0.5);

                robot.rightV4b.setPosition(0.876);
                robot.leftV4b.setPosition(0.876);

                robot.driveForCounts(25, -.5, -.5, 2000);


                robot.driveForCounts(385, -.5, .5, 3000);

                robot.driveForCounts(565, .5, .5, 3000);

                robot.driveForCounts(625, -.5, .5, 3000);


            }


        }


            // Wait for the game to start (driver presses PLAY)






            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)


            // pause for servos to move

            telemetry.addData("Path", "Complete");
            telemetry.update();

        }


    }


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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


//import org.firstinspires.ftc.robotcontroller.external.samples.HardwareTheta;


@Autonomous(name="LiftAutoTest", group="Auton")
//@Disabled
public class LiftAutoTest extends LinearOpMode {

    LinearOpMode op = this;
    /* Declare OpMode members. */
    ThetaHardware robot = new ThetaHardware(this);   // Use a Theta's hardware

    Constants constants = new Constants();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    int runloop = 1;
    double scoringPos;
    int v4bPos;
    boolean v4bDown;
    int lowPole;
    static int target = 0;
    private final double ticks_in_degrees = 384.5 / 360;
    int lift = 0;
    int liftPos = 0;
    double liftPower = 0.5;

    PIDController controller = new PIDController(p,i, d);

    public static double p = 0.018, i = 0, d = 0.001;
    public static double f = 0.075;


    ElapsedTime clawTime = new ElapsedTime();
    ElapsedTime v4bTime = new ElapsedTime();
    ElapsedTime liftTime = new ElapsedTime();
    ElapsedTime clawOpenTime = new ElapsedTime();
    ElapsedTime liftWaitTime = new ElapsedTime();



    @Override
    public void runOpMode() {




        controller = new PIDController(p, i, d);



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

        robot.rightV4b.setPosition(0.81);
        robot.leftV4b.setPosition(0.81);
        liftPos = 0;


        robot.rightClaw.setPosition(0.2);
        robot.leftClaw.setPosition(0.47);

        clawTime.reset();
        v4bTime.reset();
        liftTime.reset();
        clawOpenTime.reset();
        liftWaitTime.reset();

        waitForStart();

        while (opModeIsActive()) {


        }




        telemetry.addData("Path", "Complete");
        telemetry.update();


    }


            // Wait for the game to start (driver presses PLAY)






            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)


            // pause for servos to move




//        }


}


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

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.util.Log;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class ThetaHardware {
    /* Public OpMode members. */
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    public DcMotor liftMain;
    public DcMotor liftOther;
    public Servo rightH;
    public Servo leftH;
    public Servo rightV4b;
    public Servo leftV4b;
    public Servo rightClaw;
    public Servo leftClaw;

    public Telemetry t;





    private OpenCvCamera camera;
    private AprilTagDetectionPipeline aprilTagDetectionPipeline;


    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    private ElapsedTime waitTime = new ElapsedTime();
    LinearOpMode thOpMode;


    ElapsedTime runtime = new ElapsedTime();

    //imu
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();


    /* Constructor */
    public ThetaHardware(LinearOpMode op) {
        this.thOpMode = op;
    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry t) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        telemetry = t;

        // Define and Initialize Motors
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");

        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");

        liftMain = hwMap.get(DcMotor.class, "liftMain");
        liftOther = hwMap.get(DcMotor.class, "liftOther");

        //servos

        rightH = hwMap.get(Servo.class, "rightHoriz");
        leftH = hwMap.get(Servo.class, "leftHoriz");
        rightV4b = hwMap.get(Servo.class, "rightV4b");
        leftV4b = hwMap.get(Servo.class, "leftV4b");
        rightClaw = hwMap.get(Servo.class, "rightClaw");
        leftClaw = hwMap.get(Servo.class, "leftClaw");


        //set input mode



        frontLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        backLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        liftMain.setPower(0);
        liftOther.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftOther.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftOther.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        telemetry.addData("Status", "init Complete");    //

    }


    public void setDrivePower(double left, double right) {
        frontLeft.setPower(left);
        backLeft.setPower(left);
        frontRight.setPower(left);
        backRight.setPower(left);
    }

    public void initMotors() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stopMotors() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }


    public void strafeForCounts(int counts, double leftPower, double rightPower, int maxMS) {
        int frontLeftcounts = 0, frontRightcounts = 0, backLeftcounts = 0, backRightcounts = 0;
        boolean done = false;

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        backRight.setPower(leftPower);
        frontRight.setPower(rightPower);
        backLeft.setPower(rightPower);
        frontLeft.setPower(leftPower);


        waitTime.reset();

        while ((waitTime.milliseconds() < maxMS) && thOpMode.opModeIsActive() && (Math.abs(frontLeftcounts) < counts)) {
            frontLeftcounts = frontLeft.getCurrentPosition();
            frontRightcounts = frontRight.getCurrentPosition();
            backLeftcounts = backLeft.getCurrentPosition();
            backRightcounts = backRight.getCurrentPosition();
        }

        setDrivePower(0, 0);


    }

    public void thetaWait(double seconds) {
        runtime.reset();
        while (this.thOpMode.opModeIsActive() && (runtime.seconds() < seconds)) {

        }
    }





   public void driveForCounts( int counts, double left, double right, int maxMS )
{
    int frontLeftcounts = 0, frontRightcounts = 0, backLeftcounts = 0, backRightcounts = 0;

    frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    frontLeft.setPower(-left);
    backLeft.setPower(left);
    frontRight.setPower(right);
    backRight.setPower(-right);

    waitTime.reset();

    while((waitTime.milliseconds() < maxMS) && thOpMode.opModeIsActive() && (Math.abs(frontLeftcounts) < counts)) {
        frontLeftcounts = frontLeft.getCurrentPosition();
        frontRightcounts = frontRight.getCurrentPosition();
        backLeftcounts = backLeft.getCurrentPosition();
        backRightcounts = backRight.getCurrentPosition();
    }

    setDrivePower(0,0);


    }











}




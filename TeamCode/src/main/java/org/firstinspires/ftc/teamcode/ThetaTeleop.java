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

import android.util.Log;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwareTheta;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@Config
@TeleOp(name="Theta: Teleop POV", group="Teleop")
//@Disabled
public class ThetaTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    ThetaHardware robot = new ThetaHardware(this);   // Use a Theta's hardware

    double slowmultiplier;
    boolean startDown, startPressed;
    boolean dpadDown, dpadPressed;
    double servoPosition, servoPosition2;
    double scoringPos;
    int v4bPos;
    boolean v4bDown;
    int lowPole;
    static int target = 0;
    private final double ticks_in_degrees = 384.5 / 360;
    int liftPos = 0;
    int coneStackPos = 0;

    PIDController controller;

    public static double p = 0.018, i = 0, d = 0.001;
    public static double f = 0.075;


    ElapsedTime clawTime = new ElapsedTime();
    ElapsedTime v4bTime = new ElapsedTime();
    ElapsedTime liftTime = new ElapsedTime();
    ElapsedTime clawOpenTime = new ElapsedTime();
    ElapsedTime liftWaitTime = new ElapsedTime();

    @Override
    public void runOpMode() {


        Log.d("10695", "runOpMode: a");

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, telemetry);

        controller = new PIDController(p, i, d);

        Log.d("10695", "runOpMode: b");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //


        robot.rightV4b.setPosition(0.866);
        robot.leftV4b.setPosition(0.866);


        robot.rightClaw.setPosition(0.37);
        robot.leftClaw.setPosition(0.31);
        robot.leftH.setPosition(0.61);
        robot.rightH.setPosition(0.64);

        clawTime.reset();
        v4bTime.reset();
        liftTime.reset();
        clawOpenTime.reset();
        liftWaitTime.reset();

        scoringPos = -1;
        v4bDown = false;
        v4bPos = 1;
        liftPos = 0;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            controller.setPID(p, i, d);
            int armPos = robot.liftOther.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

            double liftPower = pid + ff;

            robot.liftOther.setPower(liftPower);
            robot.liftMain.setPower(liftPower);


            robot.frontRight.setPower((-gamepad1.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger - gamepad1.left_stick_x) * slowmultiplier);
            robot.frontLeft.setPower((gamepad1.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger - gamepad1.left_stick_x) * slowmultiplier);
            robot.backRight.setPower((gamepad1.left_stick_y - gamepad1.left_trigger + gamepad1.right_trigger - gamepad1.left_stick_x) * slowmultiplier);
            robot.backLeft.setPower((-gamepad1.left_stick_y - gamepad1.left_trigger + gamepad1.right_trigger - gamepad1.left_stick_x) * slowmultiplier);


            //LIFT POSITIONS

            //liftPos positions are as follows: 0 for Bottom, 1 for Mid, 2 for High.

            if (liftPos == 0) {
                target = 25;
            }
            if (liftPos == 1) {
                target = 925;

            }
            if (liftPos == 2) {
                target = 1690;
            }



            //SLOWMODES

            if (gamepad1.start && !startDown) {
                startPressed = !startPressed;
                startDown = true;
            } else if (!gamepad1.start) {
                startDown = false;
            }
            if (startPressed) {

                slowmultiplier = 0.5;

            } else {

                slowmultiplier = 1;

            }

            //SCORING POSITIONS

            //Scoring positions are as follows: -1
            // is a reset loop used for testing AND as a storing area for Intaking, 0 is the method to return the lift to the bottom, 1 is for Low, 2 is for Mid, 3 is for High.
            //Doubles 0.5 will be for Terminals and Ground Junctions. 1.5 will be used for scoring on low if the v4b doesn't work.
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




                        if(v4bTime.milliseconds() > 500){
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
                    robot.rightV4b.setPosition(0.40);
                    robot.leftV4b.setPosition(0.40);
                    v4bPos = 2;

                }
                if (v4bPos == 2) {
                    if(liftWaitTime.milliseconds() > 550){

                        //Raise Lift
                        liftPos = 1;
                    }
                }
                if (armPos > 250) {
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
//                    robot.rightH.setPosition(0.36);
//                    robot.leftH.setPosition(0.36);
                    v4bPos = 0;

                }
                if (v4bPos == 0) {
                    scoringPos = -1;
                }

            } else if (scoringPos == 3) {

                //Close Claw
                robot.rightClaw.setPosition(0.20);
                robot.leftClaw.setPosition(0.47);

                if (clawTime.milliseconds() > 250) {

                    //Raise Virtual 4 Bar
                    robot.rightV4b.setPosition(0.38);
                    robot.leftV4b.setPosition(0.38);
                    v4bPos = 4;
                    lowPole = 2;

                }

                if (v4bPos == 4) {
                    scoringPos = -1;
                }


            }else if (scoringPos == 4) {

                //Close Claw
                robot.rightClaw.setPosition(0.20);
                robot.leftClaw.setPosition(0.47);

                if (clawTime.milliseconds() > 250) {

                    //Raise Virtual 4 Bar
                    robot.rightV4b.setPosition(0.81);
                    robot.leftV4b.setPosition(0.81);
                    v4bPos = 4;
                    lowPole = 2;

                }

                if (v4bPos == 4) {
                    scoringPos = -1;
                }


            }





            if(coneStackPos == 1){

                robot.rightV4b.setPosition(0.77);
                robot.leftV4b.setPosition(0.77);

            }else if(coneStackPos == 2){

                robot.rightV4b.setPosition(0.79);
                robot.leftV4b.setPosition(0.79);

            }else if(coneStackPos == 3){

                robot.rightV4b.setPosition(0.81);
                robot.leftV4b.setPosition(0.81);

            }else if(coneStackPos == 4){
                robot.rightV4b.setPosition(0.83);
                robot.leftV4b.setPosition(0.83);

            }else if(coneStackPos == 5){
                coneStackPos = 0;
            }

            if (gamepad1.dpad_up && !dpadDown) {
                dpadPressed = !dpadPressed;
                dpadDown = true;
            } else if (!gamepad1.dpad_up) {
                dpadDown = false;
            }
            if (dpadPressed) {

                coneStackPos ++;
                dpadPressed = false;

            } else {

                coneStackPos = coneStackPos;

            }









                 if (gamepad1.a) {

                     scoringPos = 0;


                     coneStackPos = 0;

                 }

                if (gamepad1.b) {
                    scoringPos = 1;

                    coneStackPos = 0;
                }


                if (gamepad1.y) {
                    scoringPos = 2;

                    coneStackPos = 0;
                }


                if (gamepad1.x) {
                    scoringPos = 3;

                    coneStackPos = 0;

                }
                if (gamepad1.right_bumper) {
                    robot.rightV4b.setPosition(0.442);
                    robot.leftV4b.setPosition(0.442);
                }
                if (gamepad1.left_bumper) {
                    robot.rightV4b.setPosition(0.876);
                    robot.leftV4b.setPosition(0.876);
                }
                if(gamepad1.back){
                    scoringPos = 4;
                }



                telemetry.addData("pos ", armPos);
                telemetry.addData("target ", target);


                // Send telemetry message to signify robot running;


                telemetry.update();
                Log.d("10695", "runOpMode: 2 ");


                //If the Manifest deletes again
//
//<?xml version="1.0" encoding="utf-8"?>
//
//<!-- Note: the actual manifest file used in your APK merges this file with contributions
//            from the modules on which your app depends (such as FtcRobotController, etc).
//            So it won't ultimately be as empty as it might here appear to be :-) -->
//
//                    <!-- The package name here determines the package for your R class and your BuildConfig class -->
//<manifest
//    package="org.firstinspires.ftc.teamcode"
//            xmlns:android="http://schemas.android.com/apk/res/android">
//    <application/>
//</manifest>


            }
        }
    }




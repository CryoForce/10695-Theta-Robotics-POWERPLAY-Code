package org.firstinspires.ftc.teamcode;
//+
// .
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

// @Config
@TeleOp
public class ServoPosition extends LinearOpMode {
    public static double servoPosition = 0.5;
    public static double servo2Position =0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, "rightV4b");
        Servo servo2 = hardwareMap.get(Servo.class, "leftV4b");


        servoPosition = 0.5;
        servo2Position = 0.5;

        waitForStart();


        while (!isStopRequested()) {
            if (gamepad1.dpad_up) {
                servoPosition += 0.0003;
                servo2Position += 0.0003;
            } else if (gamepad1.dpad_down) {
                servoPosition -= 0.0003;
                servo2Position -= 0.0003;
            }

            if (gamepad1.dpad_left) {
                servo2Position += 0.0003;
            } else if (gamepad1.dpad_right) {
                servo2Position -= 0.0003;
            }



            servo.setPosition(servoPosition);
            servo2.setPosition(servo2Position);


            telemetry.addData("position", servoPosition);
            telemetry.addData("position2", servo2Position);

            telemetry.update();
        }

    }
}



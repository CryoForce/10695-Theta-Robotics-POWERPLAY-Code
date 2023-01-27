package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class PID_Arm extends OpMode {
    private PIDController controller;

    public static double p = 0.018, i =0, d = 0.001;
    public static double f = 0.05;

    public static int target = 0;

    private final double ticks_in_degrees = 384.5 / 360;

    private DcMotor arm_motor;
    private DcMotor other_motor;


    @Override
    public void init(){
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotor.class, "liftOther");
        other_motor = hardwareMap.get(DcMotor.class, "liftMain");
    }


    @Override
    public void loop(){
        controller.setPID(p, i, d);
        int armPos = arm_motor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        double power = pid + ff;

        arm_motor.setPower(power);
        other_motor.setPower(power);

        telemetry.addData("pos ", armPos);
        telemetry.addData("target ", target);
        telemetry.update();
    }

}

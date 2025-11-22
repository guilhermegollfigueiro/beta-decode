package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "aaaa", group = "Linear OpMode")
public class autonomoagr extends LinearOpMode {

    private IMU imu;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    private double i = 0.4;
    private double x = 0.0;

    private double g;

    private double c = 0;
    private double A;

    @Override
    public void runOpMode() {


        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right_motor");


        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);

        Deadline gamepadRateLimit = new Deadline(500,TimeUnit.MILLISECONDS);

        waitForStart();

        while(opModeIsActive()) {
            if (gamepadRateLimit.hasExpired() && c < 1) {
                imu.resetYaw();
                gamepadRateLimit.reset();
                c = c + 1;
            }
            g = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                telemetry.addData("graus", g);
                telemetry.update();

        }
    }
}
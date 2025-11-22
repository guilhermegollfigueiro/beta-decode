package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "AUTONOMO PARA SABADO", group = "Linear OpMode")
public class AUTONOMO_PARA_SABADO extends LinearOpMode {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    private double i = 0.2;
    private double x = 0.0;

    private double g;

    private double A;

    private IMU imu;

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

        waitForStart();


        while(opModeIsActive()) {

            frontLeft.setPower(0.4);
            backLeft.setPower(0.4);
            frontRight.setPower(0.4); //VAI PARA FREMTE
            backRight.setPower(0.4);

            sleep(400);
            telemetry.addData("Status", "Farma aura aí garoto boa sorte");
            telemetry.update();
        }
    }
}
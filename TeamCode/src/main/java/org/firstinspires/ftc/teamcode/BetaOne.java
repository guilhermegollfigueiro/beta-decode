package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name="BetaOne", group="Linear OpMode")
public class BetaOne extends LinearOpMode {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx intake1, intake2;
    private DcMotorEx shooter;
    private CRServo intake3;
    private Limelight3A limelight;
    private IMU imu;
    private double i;
    private double Lasttx;
    private int id;
    private double distance;



    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right_motor");

        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake3 = hardwareMap.get(CRServo.class, "intake3");

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        double tx = 0;
        double targetOffsetAngle_Vertical = 0;

        limelight.pipelineSwitch(0);

        waitForStart();

        limelight.setPollRateHz(100);
        limelight.start();


        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;

            double max = Math.max(Math.abs(frontLeftPower),
                    Math.max(Math.abs(backLeftPower),
                            Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));

            if (max > 1) {
                frontLeftPower /= max;
                backLeftPower /= max;
                frontRightPower /= max;
                backRightPower /= max;
            }

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                id = fiducial.getFiducialId();
            }

            if (id == 24) {
                tx = result.getTx();
                targetOffsetAngle_Vertical = result.getTy();
            }

            double limelightMountAngleDegrees = 0.0;
            double limelightLensHeightInches = 23.98 / 2.54;
            double goalHeightInches = 75 / 2.54;

            double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

            distance = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians) * 2.54;

            double Kp = 0.5;
            double error = tx;
            double turnPower = Kp * error;

            turnPower = Math.max(Math.min(turnPower, 0.2 ), -0.2);
            if (Math.abs(error) < 0.2) {
                turnPower = 0;
            }

            if (frontLeftPower != 0 &&
             backLeftPower != 0 &&
             frontRightPower != 0 &&
             backRightPower != 0) {

                    if (tx > 0.2 && (id == 20 || id == 24)) {
                        frontLeft.setPower(turnPower);
                        backLeft.setPower(turnPower);
                        frontRight.setPower(-turnPower);
                        backRight.setPower(-turnPower);
                    } else if (tx < -0.2 && (id == 20 || id == 24)) {
                        frontLeft.setPower(turnPower);
                        backLeft.setPower(turnPower);
                        frontRight.setPower(-turnPower);
                        backRight.setPower(-turnPower);
                    } else {
                    frontLeft.setPower(frontLeftPower);
                    backLeft.setPower(backLeftPower);
                    frontRight.setPower(frontRightPower);
                    backRight.setPower(backRightPower);
                }
            }

            i = -0.8;

            if (gamepad1.right_bumper) {
                shooter.setPower(i);
            } else {
                shooter.setPower(0);
            }

            if (gamepad1.a) {
                intake3.setPower(1.0);
            } else if (gamepad1.left_bumper) {
                intake1.setPower(-1.0);
                intake2.setPower(-1.0);
            } else if (gamepad1.x) {
                intake3.setPower(-1.0);
                shooter.setPower(1.0);
            } else {
                intake1.setPower(0);
                intake2.setPower(0);
                intake3.setPower(0);
            }

            telemetry.addData("Distance", distance);
            telemetry.addData("Motor Controle", i);
            telemetry.update();
        }
    }
}

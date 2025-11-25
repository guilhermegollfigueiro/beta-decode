package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name="BetaOne", group="Linear OpMode")
public class BetaOne extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx intake1, intake2;
    private DcMotorEx shooter;
    private CRServo intake3;
    private Limelight3A limelight;
    private IMU imu;
    private double i;
    private double Lasttx;
    private double A = 0.8;
    private int id;
    private double distance;



    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight = hardwareMap.get(DcMotor.class, "back_right_motor");

        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake3 = hardwareMap.get(CRServo.class, "intake3");

        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        double tx = 0;
        double targetOffsetAngle_Vertical = 0;

        limelight.pipelineSwitch(0);

        boolean last = false;

        waitForStart();

        limelight.setPollRateHz(60);
        limelight.start();

        ElapsedTime timer = new ElapsedTime();

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

            if (max > 1.0) {
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

            if (id == 20) {
                tx = result.getTx();
                targetOffsetAngle_Vertical = result.getTy();
            }

            double limelightMountAngleDegrees = 0.0;
            double limelightLensHeightInches = 23.98 / 2.54;
            double goalHeightInches = 75 / 2.54;

            double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

            distance = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians) * 2.54;

            double Kp = 0.035;
            double error = tx;
            double turnPower = Kp * error;

            turnPower = Math.max(Math.min(turnPower, 0.3), -0.3);
            if (Math.abs(error) < 0.2) {
                turnPower = 0;
            }

            if (frontLeftPower == 0 &&
             backLeftPower == 0 &&
             frontRightPower == 0 &&
             backRightPower == 0) {

                    if (id == 20) {
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
            telemetry.addData("turnpower", turnPower);

            i = -0.6572527 - 0.0005286172*distance - 0.000000709707*Math.pow(distance, 2);

            A = i;

            if (gamepad1.left_bumper) {
                intake1.setPower(-1.0);
                intake2.setPower(-1.0);

            } else {
                intake1.setPower(0);
                intake2.setPower(0);
            }

            if (gamepad1.right_bumper) {
                shooter.setPower(A);
            } else {
                shooter.setPower(0);
            }
            //CODIGO ATIRADOR DO SHOOTER
            if (gamepad1.left_bumper) {
                intake1.setPower(-1.0);
                intake2.setPower(-1.0);
            } else {
                intake1.setPower(0);
                intake2.setPower(0);
            }

            if (gamepad1.a) {
                intake3.setPower(1.0);

            } else {

                intake3.setPower(0);
            }

            telemetry.addData("Distance", distance);
            telemetry.addData("Motor Controle", i);
            telemetry.update();
        }
    }
}

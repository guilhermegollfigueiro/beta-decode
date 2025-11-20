package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.List;

@TeleOp(name="BetaOne", group="Linear OpMode")
public class BetaOne extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intake1, intake2;
    private DcMotorEx shooter;
    private CRServo intake3;
    private Limelight3A limelight;
    private IMU imu;
    private double i = 0;
    private int id;
    private double distance;

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight = hardwareMap.get(DcMotor.class, "back_right_motor");

        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake3 = hardwareMap.get(CRServo.class, "intake3");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(orientation));

        double tx = 0;
        double targetOffsetAngle_Vertical = 0;

        limelight.pipelineSwitch(0);

        double maxTicksPerSecond = 160.0 * (103.8 / 60.0);

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
            if (max > 0.8) {
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

            if (id == 20 || id == 24) {
                tx = result.getTx();
                targetOffsetAngle_Vertical = result.getTy();
            }

            // ROTACIONADOR AUTOMATICO LIMELIGHT3A
            if (result != null && result.isValid() && (gamepad1.right_trigger > 0.5)) {

                double Kp = 0.035;
                double error = tx;
                double turnPower = Kp * error;

                turnPower = Math.max(Math.min(turnPower, 0.35), -0.35);
                if (Math.abs(error) < 0.3) {
                    turnPower = 0;
                }

                double p = 0.6;

                    if (gamepad1.right_trigger > 0.3) {
                        if (result != null && (id == 20 || id == 24)) {
                            frontLeft.setPower(turnPower);
                            backLeft.setPower(turnPower);
                            frontRight.setPower(-turnPower);
                            backRight.setPower(-turnPower);
                        } else {
                            frontLeft.setPower(-p);
                            backLeft.setPower(-p);
                            frontRight.setPower(p);
                            backRight.setPower(p);
                        }
                    } else if (gamepad1.left_trigger > 0.3) {
                        if (result != null && (id == 20 || id == 24)) {
                            frontLeft.setPower(turnPower);
                            backLeft.setPower(turnPower);
                            frontRight.setPower(-turnPower);
                            backRight.setPower(-turnPower);
                        } else {
                            frontLeft.setPower(p);
                            backLeft.setPower(p);
                            frontRight.setPower(-p);
                            backRight.setPower(-p);
                        }
                    }
                }


            // CALCULADOR DE DISTANCIA LIMELIGHT3A
            double limelightMountAngleDegrees = 0.0;
            double limelightLensHeightInches = 27.5 / 2.54;
            double goalHeightInches = 74 / 2.54;

            double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

            distance = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians) * 2.54;

            double KpDistance = -0.1;
            double desiredDistance = 100.0;
            double currentDistance = distance;

            if (gamepad1.x) {
                i = i + 0.0001;
            }

            if (gamepad1.b) {
                i = i - 0.0001;
            }

            if (gamepad1.a) {
                shooter.setPower(i);
            } else {
                shooter.setPower(0);
            }

            // CODIGO ATIRADOR DO SHOOTER
            if (gamepad1.right_bumper) {
                intake1.setPower(-1.0);
                intake2.setPower(-1.0);
                intake3.setPower(1);
            } else if (gamepad1.left_bumper) {
                intake1.setPower(-1.0);
                intake2.setPower(-1.0);
                intake3.setPower(-1);
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

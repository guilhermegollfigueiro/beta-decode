package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="Intake2 + Shooter", group="Linear OpMode")
public class Intake2eShooter extends LinearOpMode {

    private Limelight3A limelight;
    private IMU imu;
    private DcMotor intake1 = null;
    private DcMotor intake2 = null;
    private DcMotor shooter = null;

    private CRServo intake3;

    double i = 0;

    private double distance;



    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake3 = hardwareMap.get(CRServo.class, "intake3");

        waitForStart();

        limelight.start();


        while (opModeIsActive()) {

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());
            LLResult llResult = limelight.getLatestResult();

            if (llResult != null && llResult.isValid()) {
                double targetOffsetAngle_Vertical = llResult.getTy();

                // Parâmetros da Limelight
                double limelightMountAngleDegrees = 0.0;
                double limelightLensHeightInches = 27.5 / 2.54;
                double goalHeightInches = 74 / 2.54;

                // Cálculo da distância
                double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
                double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
                distance = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
            }
                if (gamepad1.x) {
                    i = i + 0.05;
                }
                if (gamepad1.b) {
                    i = i - 0.05;
                }

                if (gamepad1.right_bumper) {
                    shooter.setPower(i);
                } else {
                    shooter.setPower(0);
                }

                if (gamepad1.left_bumper) {
                    intake3.setPower(1);
                } else {
                    intake3.setPower(0);
                }

                if(gamepad1.y) {
                    intake1.setPower(-1);
                    intake2.setPower(-1);
                } else {
                    intake1.setPower(0);
                    intake2.setPower(0);
                }
            }
            telemetry.addData("Valor motor:", i);
            telemetry.addData("Distancia", distance);
            telemetry.update();
        }
    }

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="MovimentadorV2", group="Linear OpMode")
public class automovimentov2 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;


    private Limelight3A limelight;
    private IMU imu;

    private boolean movingToTarget = false;

    private double distance;

    @Override
    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        waitForStart();
        runtime.reset();

        limelight.start();


        while (opModeIsActive()) {

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());
            LLResult llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                Pose3D botPose = llResult.getBotpose();
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addData("Distance", distance);
                double targetOffsetAngle_Vertical = llResult.getTy();

                // how many degrees back is your limelight rotated from perfectly vertical?
                double limelightMountAngleDegrees = 0.0;

                // distance from the center of the Limelight lens to the floor
                double limelightLensHeightInches = 5/2.54;

                // distance from the target to the floor
                double goalHeightInches = 25/2.54;

                double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
                double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

                //calculate distance
                distance = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians) * 2.54;

                double KpDistance = -0.1;  // Constante proporcional para controle de distância
                double desiredDistance = 100.0;  // Exemplo de distância desejada em centímetros

                double currentDistance = distance;

                if (gamepad1.a && !movingToTarget) {
                    movingToTarget = true;
                }

                if (movingToTarget) {
                    double Kp = 0.02;
                    double Ke = 0.04;
                    double minPower = 0.08;

                    double distanceError = desiredDistance - distance;

                    double drivingAdjust = Kp * distanceError * Math.exp(-Ke * Math.abs(distanceError));
                    drivingAdjust += Math.signum(distanceError) * minPower;

                    drivingAdjust = Math.max(Math.min(drivingAdjust, 0.4), -0.4);

                    if (Math.abs(distanceError) < 3) { // chegou perto
                        movingToTarget = false;
                        drivingAdjust = 0;
                    }

                    frontLeftDrive.setPower(drivingAdjust);
                    backLeftDrive.setPower(drivingAdjust);
                    frontRightDrive.setPower(drivingAdjust);
                    backRightDrive.setPower(drivingAdjust);
                } else {
                    frontLeftDrive.setPower(0);
                    backLeftDrive.setPower(0);
                    frontRightDrive.setPower(0);
                    backRightDrive.setPower(0);
                }

                telemetry.update();
            }
        }}}
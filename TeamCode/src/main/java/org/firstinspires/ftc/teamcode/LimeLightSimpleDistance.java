package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="Distance Calc", group="Demo")
public class LimeLightSimpleDistance extends OpMode {

    private Limelight3A limelight;
    private IMU imu;

    private double distance;

    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }
    @Override
    public void start(){
        limelight.start();
    }
    @Override
    public void loop(){
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
            double limelightLensHeightInches = 0;

            // distance from the target to the floor
            double goalHeightInches = 14.5;

            double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

            //calculate distance
            distance = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        }
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="Distance Calc", group="Demo")
public class AutoApriltagMover extends OpMode {

    private Limelight3A limelight;
    private IMU imu;

    // Motores do chassi (supondo um chassi diferencial — 2 motores)
    private DcMotor leftDrive;
    private DcMotor rightDrive;

    private double distance;
    private double desired_distance = 39.37; // distância desejada em polegadas

    // Constante de controle proporcional
    private final float KpDistance = -0.02f;  // ajuste fino da sensibilidade

    @Override
    public void init(){
        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(0);

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        // Motores do chassi
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        // Ajusta direção — dependendo de como estão montados
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Inicializado");
        telemetry.update();
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
            double targetOffsetAngle_Vertical = llResult.getTy();

            // Parâmetros da Limelight
            double limelightMountAngleDegrees = 0.0;
            double limelightLensHeightInches = 0.0;
            double goalHeightInches = 19.0;

            // Cálculo da distância
            double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
            double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
            distance = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

            // ---- Controle proporcional para distância ----
            if (gamepad1.a) { // mover ao segurar o botão A
                float current_distance = (float) distance;
                float distance_error = (float) (desired_distance - current_distance);
                float distance_adjust = KpDistance * distance_error;

                // Limita a potência máxima (para evitar trancos)
                distance_adjust = Math.max(-0.4f, Math.min(0.4f, distance_adjust));

                // Aplica o movimento
                leftDrive.setPower(distance_adjust);
                rightDrive.setPower(distance_adjust);

                telemetry.addData("Movendo", true);
                telemetry.addData("Erro de distância", distance_error);
                telemetry.addData("Ajuste aplicado", distance_adjust);
            } else {
                // Para o robô quando soltar o botão
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }

            telemetry.addData("Distance (in)", distance);
        } else {
            telemetry.addData("Status", "No valid target");
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }

        telemetry.update();
    }
}

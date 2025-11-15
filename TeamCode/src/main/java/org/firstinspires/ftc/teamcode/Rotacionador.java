package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "AlinharSuaveComLimelight", group = "Linear OpMode")
public class Rotacionador extends LinearOpMode {

    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void runOpMode() {
        // Motores
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Limelight + IMU
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientation));

        limelight.pipelineSwitch(0);

        telemetry.addData("Status", "Pronto para iniciar");
        telemetry.update();

        waitForStart();
        limelight.start();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx(); // erro horizontal em graus

                // zona morta e controle suavizado
                double deadband = 0.8;  // tolerância em graus
                double Kp = 0.03;       // ganho base
                double Ke = 0.05;       // taxa de suavização exponencial

                double error = tx;
                double absError = Math.abs(error);

                // aplica uma desaceleração exponencial conforme o erro diminui
                double turnPower = Kp * error * Math.exp(-Ke * absError);

                // limita potência máxima
                turnPower = Math.max(Math.min(turnPower, 0.25), -0.25);

                // aplica zona morta
                if (absError < deadband) {
                    turnPower = 0;
                }

                // define potência para girar suavemente
                frontLeftDrive.setPower(turnPower);
                backLeftDrive.setPower(turnPower);
                frontRightDrive.setPower(-turnPower);
                backRightDrive.setPower(-turnPower);

                telemetry.addData("tx (erro)", tx);
                telemetry.addData("Potência de giro", turnPower);
                telemetry.update();
            } else {
                // sem alvo — para tudo
                frontLeftDrive.setPower(0);
                backLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                backRightDrive.setPower(0);
                telemetry.addData("Status", "Sem alvo detectado");
                telemetry.update();
            }
        }
    }
}

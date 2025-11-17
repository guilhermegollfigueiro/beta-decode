package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "Rotacionador", group = "Linear OpMode")
public class Rotacionador extends LinearOpMode {

    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void runOpMode() {
        // Inicializa motores
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        // Direções dos motores
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Limelight e IMU
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
                double tx = result.getTx(); // deslocamento horizontal (em graus)

                // Controle proporcional simples
                double Kp = 0.035; // ajuste o ganho conforme necessário
                double error = tx;
                double turnPower = Kp * error;

                // Limita a potência para evitar sobrecorreções
                turnPower = Math.max(Math.min(turnPower, 0.3), -0.3);

                // Se o erro for pequeno, para o robô
                if (Math.abs(error) < 1.0) {
                    turnPower = 0;
                }

                // Define potência oposta nos lados para girar
                frontLeftDrive.setPower(turnPower);
                backLeftDrive.setPower(turnPower);
                frontRightDrive.setPower(-turnPower);
                backRightDrive.setPower(-turnPower);

                telemetry.addData("tx (erro)", tx);
                telemetry.addData("Potência de giro", turnPower);
                telemetry.update();
            } else {
                // Se não há alvo detectado, para os motores
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

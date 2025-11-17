package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="chassis + intake", group="Linear OpMode")
public class chassis_intake extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Motores do chassi
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    // Motor do intake
    private DcMotor intakeMotor = null;

    @Override
    public void runOpMode() {

        // Mapeamento dos motores
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        intakeMotor = hardwareMap.get(DcMotor.class, "motor");

        // Direções
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        intakeMotor.setPower(0.0);

        telemetry.addData("Status", "Inicializado — pronto para Start");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // ---------------- CONTROLE DE MOVIMENTO ----------------
            double axial   = -gamepad1.left_stick_y;  // frente/trás
            double lateral =  gamepad1.left_stick_x;  // esquerda/direita
            double yaw     =  gamepad1.right_stick_x; // rotação

            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            // Normalização
            double max = Math.max(
                    Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
            );

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // ---------------- AJUSTE DE VELOCIDADE ----------------
            // Velocidade padrão reduzida (50%)
            double speedScale = 0.5;

            // Modo turbo (pressione o botão A para 100%)
            if (gamepad1.a) {
                speedScale = 1.0;
            }

            // Aplica o fator de velocidade
            frontLeftPower  *= speedScale;
            frontRightPower *= speedScale;
            backLeftPower   *= speedScale;
            backRightPower  *= speedScale;

            // Envia pros motores
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            // ---------------- INTAKE ----------------
            if (gamepad1.left_bumper) {
                intakeMotor.setPower(1.0);
            } else if (gamepad1.right_bumper) {
                intakeMotor.setPower(-1.0);
            } else {
                intakeMotor.setPower(0.0);
            }

            // ---------------- TELEMETRIA ----------------
            telemetry.addData("Status", "Rodando: %.1f s", runtime.seconds());
            telemetry.addData("Velocidade", speedScale == 1.0 ? "TURBO" : "Normal (50%)");
            telemetry.addData("Intake Power", intakeMotor.getPower());
            telemetry.update();
        }

        // Para tudo ao sair
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        intakeMotor.setPower(0);
    }
}

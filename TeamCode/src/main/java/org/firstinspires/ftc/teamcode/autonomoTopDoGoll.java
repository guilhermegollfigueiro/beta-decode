package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "autonomoDoPai", group = "Linear OpMode")
public class autonomoTopDoGoll extends LinearOpMode {

    // Motores do chassi
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    // Motor auxiliar
    private DcMotor motor = null;

    @Override
    public void runOpMode() {
        // Mapeamento dos motores (nomes devem bater com os configurados no Driver Hub)
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        motor = hardwareMap.get(DcMotor.class, "motor");

        // Ajusta direções (isso depende de como os motores estão montados)
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Pronto para iniciar");
        telemetry.update();

        waitForStart();

        // Liga o motor extra continuamente
        motor.setPower(-1.0);

        // Move o robô para frente (todos com a mesma potência)
        double drivePower = 1.0;

        frontLeftDrive.setPower(0.2);
        backLeftDrive.setPower(-0.2);
        frontRightDrive.setPower(0.2);
        backRightDrive.setPower(-0.2);

        frontLeftDrive.setPower(drivePower);
        backLeftDrive.setPower(drivePower);
        frontRightDrive.setPower(drivePower);
        backRightDrive.setPower(drivePower);

        sleep(1000);  // anda por 2 segundos

         // Para o chassi
        frontLeftDrive.setPower(0.2);
        backLeftDrive.setPower(0.2);
        frontRightDrive.setPower(0.2);
        backRightDrive.setPower(0.2);

        sleep(2000);  // anda por 2 segundos

        // Para o chassi
        frontLeftDrive.setPower(-1.0);
        backLeftDrive.setPower(-1.0);
        frontRightDrive.setPower(-1.0);
        backRightDrive.setPower(-1.0);

        sleep(2000);  // anda por 2 segundos

        // (opcional) parar o motor extra depois
        // extraMotor.setPower(0);

        telemetry.addData("Status", "Autônomo concluído");
        telemetry.update();
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Autonomo", group = "Linear OpMode")
public class Autonomo extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private double i = 0.35;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight = hardwareMap.get(DcMotor.class, "back_right_motor");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        //PRIMEIRAS 3 BOLINHAS DENTRO DO ROBO
        frontLeft.setPower(-i);
        backLeft.setPower(-i);
        frontRight.setPower(-i); //VAI PARA TRAS
        backRight.setPower(-i);

        sleep(1400);

        frontLeft.setPower(-i);
        backLeft.setPower(-i);
        frontRight.setPower(i); // ROTACIONA PARA ESQUERDA
        backRight.setPower(i);

        sleep(340);

        frontLeft.setPower(-i);
        backLeft.setPower(i);
        frontRight.setPower(i); //ANDA PARA ESQUERDA
        backRight.setPower(-i);

        sleep(80);

        //PRIMEIRO INTAKE 3 BOLINHAS
        frontLeft.setPower(i);
        backLeft.setPower(i);
        frontRight.setPower(i); //ANDA PARA FRENTE
        backRight.setPower(i);

        sleep(1100);

        frontLeft.setPower(-i);
        backLeft.setPower(-i);
        frontRight.setPower(-i); //ANDA PARA TRÁS
        backRight.setPower(-i);

        sleep(1100);

        frontLeft.setPower(i);
        backLeft.setPower(-i);
        frontRight.setPower(-i);  //ANDA PARA DIREITA
        backRight.setPower(i);

        sleep(80);

        frontLeft.setPower(i);
        backLeft.setPower(i);
        frontRight.setPower(-i); //ROTACIONA PARA DIREITA
        backRight.setPower(-i);

        sleep(340);

        //SEGUNDO INTAKE 3 BOLINHAS
        frontLeft.setPower(-i);
        backLeft.setPower(-i);
        frontRight.setPower(i); //ROTACIONA PARA ESQUERDA
        backRight.setPower(i);

        sleep(340);

        frontLeft.setPower(-i);
        backLeft.setPower(i);
        frontRight.setPower(i); //ANDA PARA ESQUERDA
        backRight.setPower(-i);

        sleep(760);

        frontLeft.setPower(i);
        backLeft.setPower(i);
        frontRight.setPower(i); //ANDA PARA FRENTE
        backRight.setPower(i);

        sleep(1100);

        frontLeft.setPower(-i);
        backLeft.setPower(-i);
        frontRight.setPower(-i); //ANDA PARA TRÁS
        backRight.setPower(-i);

        sleep(1100);

        frontLeft.setPower(i);
        backLeft.setPower(-i);
        frontRight.setPower(-i); //ANDA PARA DIREITA
        backRight.setPower(i);

        sleep(760);

        frontLeft.setPower(i);
        backLeft.setPower(i);
        frontRight.setPower(-i); //ROTACIONA PARA DIREITA
        backRight.setPower(-i);

        sleep(340);

        //TERCEIRO INTAKE 3 BOLINHAS
        frontLeft.setPower(-i);
        backLeft.setPower(-i);
        frontRight.setPower(i); //ROTACIONA PARA ESQUERDA
        backRight.setPower(i);

        sleep(340);

        frontLeft.setPower(-i);
        backLeft.setPower(i);
        frontRight.setPower(i); //ANDA PARA ESQUERDA
        backRight.setPower(-i);

        sleep(1470);

        frontLeft.setPower(i);
        backLeft.setPower(i);
        frontRight.setPower(i); //ANDA PARA FRENTE
        backRight.setPower(i);

        sleep(1100);

        frontLeft.setPower(-i);
        backLeft.setPower(-i);
        frontRight.setPower(-i); //ANDA PARA TRÁS
        backRight.setPower(-i);

        sleep(1100);

        frontLeft.setPower(i);
        backLeft.setPower(-i);
        frontRight.setPower(-i); //ANDA PARA DIREITA
        backRight.setPower(i);

        sleep(1470);

        frontLeft.setPower(i);
        backLeft.setPower(i);
        frontRight.setPower(-i); //ROTACIONA PARA DIREITA
        backRight.setPower(-i);

        sleep(340);

        frontLeft.setPower(-i);
        backLeft.setPower(-i);
        frontRight.setPower(i); //ROTACIONA PARA ESQUERDA
        backRight.setPower(i);

        sleep(340);

        frontLeft.setPower(i);
        backLeft.setPower(i);
        frontRight.setPower(i); //ANDA PARA FRENTE
        backRight.setPower(i);

        sleep(900);

        frontLeft.setPower(-i);
        backLeft.setPower(i);
        frontRight.setPower(i); //ANDA PARA ESQUERDA
        backRight.setPower(-i);

        sleep(260);

        telemetry.addData("Status", "Autônomo concluído");
        telemetry.update();
    }
}
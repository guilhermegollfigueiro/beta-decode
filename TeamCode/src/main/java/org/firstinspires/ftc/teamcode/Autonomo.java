package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Autonomo", group = "Linear OpMode")
public class Autonomo extends LinearOpMode {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    private double i = 0.2;
    private double x = 0.0;

    private double g;

    private double A;

    private IMU imu;

    @Override
    public void runOpMode() {

        IMUbeta bench = new IMUbeta();



        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right_motor");


        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);

        waitForStart();

        bench.init(hardwareMap);

        while(opModeIsActive()) {
            g = bench.getHeading(AngleUnit.DEGREES) + 180;
            A = 40.5 + 180;
            double Kp = 0.02;
            double error = A;
            double correcao = Kp * A;

            correcao = Math.max(Math.min(correcao, 0.2 ), -0.2);
            if (Math.abs(g) < Math.abs(A) + 0.3) {
                correcao = 0;
            }

            //PRIMEIRAS 3 BOLINHAS DENTRO DO ROBO
            if (g < A) {
                frontLeft.setPower(0.5 - correcao);
                backLeft.setPower(0.5 + correcao);
                frontRight.setPower(0.5 + correcao); //VAI PARA FRENTE E CORRIGE NA ESQUERDA
                backRight.setPower(0.5 - correcao);
            } else {
                frontLeft.setPower(0.5 + correcao);
                backLeft.setPower(0.5 - correcao);
                frontRight.setPower(0.5 - correcao); //VAI PARA FRENTE E CORRIGE NA ESQUERDA
                backRight.setPower(0.5 + correcao);
            }
            sleep(1000);

            frontLeft.setPower(x);
            backLeft.setPower(x);
            frontRight.setPower(x); //VAI PARA TRAS
            backRight.setPower(x);

            sleep(500);

            frontLeft.setPower(-0.5);
            backLeft.setPower(-0.5);
            frontRight.setPower(-0.5); //VAI PARA TRAS
            backRight.setPower(-0.5);

            sleep(1500);

            //PRIMEIRAS 3 BOLINHAS DENTRO DO ROBO
            frontLeft.setPower(-i);
            backLeft.setPower(-i);
            frontRight.setPower(-i); //VAI PARA TRAS
            backRight.setPower(-i);

            sleep(2800);

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
}
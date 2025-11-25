package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Autonomo Atrás Azul", group = "Linear OpMode")
public class AutonomoAtrasAzul extends LinearOpMode {

    private IMU imu;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    private double i = 0.2;
    private double x = 0.0;
    private DcMotorEx intake1, intake2;
    private DcMotorEx shooter;
    private CRServo intake3;
    private double g;


    private double r = 0.35; //FORÇA DAS ROTAÇÕES
    private double c = 0;
    private double A;

    @Override
    public void runOpMode() {


        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right_motor");

        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake3 = hardwareMap.get(CRServo.class, "intake3");

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);

        Deadline gamepadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);

        waitForStart();

        while (opModeIsActive() && x < 1) {
            x = x + 1;
            if (gamepadRateLimit.hasExpired() && c < 1) {
                imu.resetYaw();
                gamepadRateLimit.reset();
                c = c + 1;
            }
            ElapsedTime timer = new ElapsedTime();

            while (timer.seconds() < 3) {
                shooter.setPower(-1);
                telemetry.addData("A", A); // ATIRA NA CESTA
                telemetry.addData("g", g);
                telemetry.update();
            }

            timer.reset();
            while (timer.seconds() < 5) {
                shooter.setPower(-0.95);
                intake1.setPower(-0.7);
                intake2.setPower(-0.7);
                intake3.setPower(1);
                telemetry.addData("A", A); // ATIRA NA CESTA
                telemetry.addData("g", g);
                telemetry.update();
            }
            intake1.setPower(0);
            intake2.setPower(0);
            intake3.setPower(0);
            shooter.setPower(0);

            timer.reset();
            g = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            A = -120;
            while (timer.seconds() < 2.4) {
                g = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                double Kp = 0.01;
                double error = Math.abs(g - A);
                double correcao = Kp * error;

                correcao = Math.max(Math.min(correcao, r), -r);
                if (g < A) {
                    frontLeft.setPower(-correcao);
                    backLeft.setPower(-correcao);
                    frontRight.setPower(correcao); //VAI PARA FRENTE E CORRIGE NA ESQUERDA
                    backRight.setPower(correcao);
                } else if (g > A) {
                    frontLeft.setPower(correcao);
                    backLeft.setPower(correcao);
                    frontRight.setPower(-correcao); //VAI PARA FRENTE E CORRIGE NA DIREITA
                    backRight.setPower(-correcao);
                } else {
                    frontLeft.setPower(0);
                    backLeft.setPower(0);
                    frontRight.setPower(0); //VAI PARA FRENTE
                    backRight.setPower(0);
                }
                telemetry.addData("A", A);
                telemetry.addData("g", g);
                telemetry.addData("correcao", correcao);
                telemetry.update();
            }
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
            sleep(200);

            timer.reset();
            g = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            A = -120;
            while (timer.seconds() < 1.1) {
                g = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                double Kp = 0.015;
                double error = g - A;
                double correcao = Math.abs(Kp * error);

                correcao = Math.max(Math.min(correcao, 0.03), -0.03);
                if (g < A + 0.3 && g > A - 0.3) {
                    correcao = 0;
                }

                if (g < A) {
                    frontLeft.setPower(0.25 - correcao);
                    backLeft.setPower(0.25 - correcao);
                    frontRight.setPower(0.25 + correcao); //VAI PARA FRENTE E CORRIGE NA ESQUERDA
                    backRight.setPower(0.25 + correcao);
                } else if (g > A) {
                    frontLeft.setPower(0.25 + correcao);
                    backLeft.setPower(0.25 + correcao);
                    frontRight.setPower(0.25 - correcao); //VAI PARA FRENTE E CORRIGE NA DIREITA
                    backRight.setPower(0.25 - correcao);
                } else {
                    frontLeft.setPower(0.25);
                    backLeft.setPower(0.25);
                    frontRight.setPower(0.25); //VAI PARA FRENTE
                    backRight.setPower(0.25);
                }
                intake1.setPower(-1);
                intake2.setPower(-1);
                telemetry.addData("A", A);
                telemetry.addData("g", g);
                telemetry.addData("correcao", correcao);
                telemetry.update();
            }
            intake1.setPower(0);
            intake2.setPower(0);
            intake3.setPower(0);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
            sleep(200);
        }
    }
}
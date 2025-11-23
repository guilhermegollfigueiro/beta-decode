package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Autonomo PID", group = "Linear OpMode")
public class autonomoduvido extends LinearOpMode {

    private IMU imu;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx intake1, intake2, shooter;
    private CRServo intake3;

    private double shooterPower = -0.78;

    // ---------------------------
    // CLASSE DE PID ANGULAR
    // ---------------------------
    public class PID {
        private double kP, kI, kD;
        private double integral = 0;
        private double lastError = 0;

        public PID(double p, double i, double d) {
            kP = p;
            kI = i;
            kD = d;
        }

        public double calculate(double error) {
            integral += error;
            double derivative = error - lastError;
            lastError = error;
            return kP * error + kI * integral + kD * derivative;
        }
    }

    // ---------------------------
    // ERRO CIRCULAR (EVITA 359→0 BUG)
    // ---------------------------
    private double angleError(double target, double current) {
        double error = target - current;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }

    // ---------------------------
    // FUNÇÃO PARA GIRAR AO ÂNGULO
    // ---------------------------
    private void turnTo(double targetAngle, double maxPower, double timeout) {

        PID pid = new PID(0.012, 0.00003, 0.002);
        ElapsedTime t = new ElapsedTime();

        while (opModeIsActive() && t.seconds() < timeout) {

            double g = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = angleError(targetAngle, g);

            double out = pid.calculate(error);
            out = Range.clip(out, -maxPower, maxPower);

            frontLeft.setPower(-out);
            backLeft.setPower(-out);
            frontRight.setPower(out);
            backRight.setPower(out);

            telemetry.addData("Target", targetAngle);
            telemetry.addData("Angle", g);
            telemetry.addData("Error", error);
            telemetry.addData("Out", out);
            telemetry.update();
        }

        stopDrive();
    }

    // ---------------------------
    // DIRIGIR RETO COM ÂNGULO TRAVADO
    // ---------------------------
    private void driveStraight(double tempo, double basePower, double angle) {
        PID pid = new PID(0.01, 0, 0.001);

        ElapsedTime t = new ElapsedTime();

        while (opModeIsActive() && t.seconds() < tempo) {

            double g = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = angleError(angle, g);

            double adjust = pid.calculate(error);

            double left = basePower - adjust;
            double right = basePower + adjust;

            frontLeft.setPower(left);
            backLeft.setPower(left);
            frontRight.setPower(right);
            backRight.setPower(right);

            telemetry.addData("Angle", g);
            telemetry.addData("Adj", adjust);
            telemetry.update();
        }

        stopDrive();
    }

    // ---------------------------
    // PARAR
    // ---------------------------
    private void stopDrive() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    // ---------------------------
    // AUTÔNOMO
    // ---------------------------
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

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));

        waitForStart();

        imu.resetYaw();

        // -------------------------
        // 1 — ANDAR FRENTE (1.4s) + SHOOT
        // -------------------------
        shooter.setPower(shooterPower);
        driveStraight(1.2, 0.4, 0);

        sleep(2000);

        ElapsedTime shootTimer = new ElapsedTime();
        while (shootTimer.seconds() < 5) {
            intake1.setPower(-0.7);
            intake2.setPower(-0.7);
            intake3.setPower(1);
        }
        intake1.setPower(0);
        intake2.setPower(0);
        intake3.setPower(0);
        shooter.setPower(0);

        // -------------------------
        // 2 — GIRAR PARA -137.88
        // -------------------------
        turnTo(-137.88, 0.35, 2);

        // -------------------------
        // 3 — INDO PARA O INTAKE (1)
        // -------------------------
        driveStraight(1.8, 0.25, -137.88);
        intake1.setPower(0);
        intake2.setPower(0);

        // -------------------------
        // 4 — VOLTANDO COM A BOLA
        // -------------------------
        driveStraight(1.8, -0.25, -137.88);

        // -------------------------
        // 5 — VOLTA PARA 0° E ATIRA
        // -------------------------
        shooter.setPower(shooterPower);
        turnTo(0, 0.35, 3);

        shootTimer.reset();
        while (shootTimer.seconds() < 7) {
            intake1.setPower(-0.7);
            intake2.setPower(-0.7);
            intake3.setPower(1);
        }
        intake1.setPower(0);
        intake2.setPower(0);
        intake3.setPower(0);
        shooter.setPower(0);

        // -------------------------
        // 6 — GIRAR PARA -46.24
        // -------------------------
        turnTo(-46.24, 0.35, 2);

        // -------------------------
        // 7 — ANDAR ATÉ 2º INTAKE
        // -------------------------
        driveStraight(0.6, 0.25, -46.24);

        // -------------------------
        // 8 — GIRAR PARA -137.88
        // -------------------------
        turnTo(-137.88, 0.35, 2);

        telemetry.addData("Status", "Autônomo Finalizado!");
        telemetry.update();
        sleep(1000);
    }
}

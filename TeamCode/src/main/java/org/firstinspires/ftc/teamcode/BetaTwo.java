package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="BetaTwo", group="Linear OpMode")
public class BetaTwo extends LinearOpMode {

    /*CONTROLES:
      Joystick = move o robô
      LB = liga os intakes e desliga o shooter/ desliga os intakes e lizga o shooter
      RB = executa a ação de atirar as duas bolinhas pegas de uma vez
      B = rotaciona o robô em direção ao april tag
      Y = distancia o robô para a posição em que sempre acertará (ISSO VAI DEIXAR DE EXISTIR
      POIS SERÁ TROCADO PELO CÁLCULO DE FORÇA DO MOTOR)

      obs programador: trocar os nomes do motores no codigo e dos intakes e shooter no driver hub
     */
    private DcMotor frontLeft, frontRight, backLeft, backRight = null;

    private DcMotor intake1, intake2, shooter = null;

    private CRServo intake3 = null;

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight = hardwareMap.get(DcMotor.class, "back_right_motor");

        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        intake3 = hardwareMap.get(CRServo.class, "intake3");

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();


        while (opModeIsActive()) {


            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;

            double max = Math.max(Math.abs(frontLeftPower),
                    Math.max(Math.abs(backLeftPower),
                            Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));
            if (max > 1.0) {
                frontLeftPower /= max;
                backLeftPower /= max;
                frontRightPower /= max;
                backRightPower /= max;
            }

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);


            //CODIGO ALTERNADOR ENTRE INTAKES E SHOOTER
            if (gamepad1.left_bumper) {
                intake1.setPower(-1.0);
                intake2.setPower(-1.0);

            } else {
                intake1.setPower(0);
                intake2.setPower(0);
            }

            if (gamepad1.right_bumper) {
                shooter.setPower(-0.9);
            } else {
                shooter.setPower(0);
            }
            //CODIGO ATIRADOR DO SHOOTER
            if (gamepad1.left_bumper) {
                intake1.setPower(-1.0);
                intake2.setPower(-1.0);
            } else {
                intake1.setPower(0);
                intake2.setPower(0);
            }

            if (gamepad1.a) {
                intake3.setPower(1.0);

            } else {

                intake3.setPower(0);
            }

            telemetry.update();
        }
    }
}

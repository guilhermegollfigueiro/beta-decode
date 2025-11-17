package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="OBS: ESSE CODIGO NAO FUNCIONA", group="Linear OpMode")
public class OmnidirecionalTrigonometrico extends LinearOpMode {

    //OBSERVAÇÃO, NADA DESSE CÓDIGO FUNCIONA, VOU ARUMAR DEPOIS PARA DEIXÁ-LO FUNCIONAL
    //NAO DELETE ESSE CÓDIGO
    private DcMotor frontLeft, frontRight, backLeft, backRight = null;

    // Variáveis que estavam faltando
    double sin = 0;
    double cos = 0;
    double max = 0;

    double power = 0;
    double turn = 0;
    double x = 0;
    double y = 0;
    double theta = 0;

    @Override
    public void runOpMode() {

        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Direções corretas para X-Drive 45°
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Freio (para não deslizar)
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            sin = Math.sin(theta - Math.PI/4);
            cos = Math.cos(theta - Math.PI/4);
            max = Math.max(Math.abs(sin), Math.abs(cos));

            double frontLeft = power * cos/max + turn;
            double frontRight = power * cos/max - turn;
            double  backLeft = power * sin/max + turn;
            double backRight = power * cos/max - turn;

            if ((power + Math.abs(turn)) > 1) {
                frontLeft /= power + turn;
                frontRight /= power + turn;
                backLeft /= power + turn;
                backRight /= power + turn;
            }

            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
        }
    }
}

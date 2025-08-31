package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="linear_guide", group="Linear Opmode")
public class linear_guide extends LinearOpMode {

    private DcMotor motorEsquerdo;
    private DcMotor motorDireito;

    @Override
    public void runOpMode() {
        motorEsquerdo = hardwareMap.get(DcMotor.class, "motorEsquerdo");
        motorDireito = hardwareMap.get(DcMotor.class, "motorDireito");

        motorDireito.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        ElapsedTime tempoR1 = new ElapsedTime();
        ElapsedTime tempoL1 = new ElapsedTime();

        boolean r1Pressionado = false;
        boolean l1Pressionado = false;

        while (opModeIsActive()) {
            boolean r1 = gamepad1.right_bumper;
            boolean l1 = gamepad1.left_bumper;

            if (r1) {
                if (!r1Pressionado) {
                    tempoR1.reset();
                    r1Pressionado = true;
                }

                if (tempoR1.seconds() <= 1.0) {
                    motorEsquerdo.setPower(0.5);
                    motorDireito.setPower(-0.5);
                } else {
                    motorEsquerdo.setPower(0);
                    motorDireito.setPower(0);
                }
            }

            else if (l1) {
                if (!l1Pressionado) {
                    tempoL1.reset();
                    l1Pressionado = true;
                }

                if (tempoL1.seconds() <= 1.0) {
                    motorEsquerdo.setPower(-0.5);
                    motorDireito.setPower(0.5);
                } else {
                    motorEsquerdo.setPower(0);
                    motorDireito.setPower(0);
                }
            }

            else {
                r1Pressionado = false;
                l1Pressionado = false;
                motorEsquerdo.setPower(0);
                motorDireito.setPower(0);
            }

            idle();
        }
    }
}
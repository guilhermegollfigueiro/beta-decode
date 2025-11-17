package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="ControleOpMode", group="Linear OpMode")

public class Intake1 extends LinearOpMode {
    private DcMotor intake1 = null;

    @Override
    public void runOpMode() {

        intake1 = hardwareMap.get(DcMotor.class, "intake1");

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.left_bumper) {
                intake1.setPower(1.0);
             } else {
                intake1.setPower(0);
            }
            }
        }
    }

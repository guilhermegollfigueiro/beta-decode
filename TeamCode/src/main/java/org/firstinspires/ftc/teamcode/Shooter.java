package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Shooter", group="Linear OpMode")
public class Shooter extends LinearOpMode {
    private DcMotor shooter = null;

    @Override
    public void runOpMode() {

        shooter = hardwareMap.get(DcMotor.class, "shooter");

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.right_bumper) {
                shooter.setPower(0.85);
            } else {
                shooter.setPower(0);
            }
        }
    }
}

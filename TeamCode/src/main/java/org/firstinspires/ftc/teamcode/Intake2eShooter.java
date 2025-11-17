package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Intake2 + Shooter", group="Linear OpMode")
public class Intake2eShooter extends LinearOpMode {
    private DcMotor intake2 = null;
    private DcMotor shooter = null;

    @Override
    public void runOpMode() {

        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.left_bumper) {
                shooter.setPower(0.85);
                sleep(2000);
                intake2.setPower(1);
            } else {
                intake2.setPower(0);
                shooter.setPower(0);
            }
        }
    }
}

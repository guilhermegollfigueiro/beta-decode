package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="claw", group="Linear Opmode")
public class claw extends LinearOpMode {

    private Servo servo;
    boolean toggleState = false;
    boolean lastButtonState = false;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "servo");

        // Define posição inicial (0°)
        servo.setPosition(0.0);

        telemetry.addData("Status", "Pronto");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean buttonState = gamepad1.right_bumper;

            // Detecta clique (não segurar)
            if (buttonState && !lastButtonState) {
                toggleState = !toggleState;

                if (toggleState) {
                    servo.setPosition(0.5);
                } else {
                    servo.setPosition(0.0);
                }
            }

            lastButtonState = buttonState;

            telemetry.addData("Servo pos", servo.getPosition());
            telemetry.update();
        }
    }
}

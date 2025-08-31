package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "limelight3A", group = "Linear Opmode")
public class limelight3A extends OpMode {

    private Limelight3A limelight;

    @Override
    public void init() {
        // O nome "limelight" deve ser igual ao configurado no Robot Controller (Configure Robot → Add → Limelight3A)
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);

        limelight.start();

        telemetry.addLine("Limelight 3A inicializada. Abra o stream web para conferir a pipeline.");
        telemetry.update();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double tx = result.getTx();
            double ty = result.getTy();
            double ta = result.getTa();

            telemetry.addData("tx (deg)", "%.2f", tx);
            telemetry.addData("ty (deg)", "%.2f", ty);
            telemetry.addData("ta (%%)", "%.2f", ta);

            telemetry.addData("Pipeline", result.getPipelineIndex());
            telemetry.addData("Staleness (ms)", result.getStaleness()); // quão “velho” está o dado
            telemetry.addData("Tem alvo?", result.isValid() ? "SIM" : "NAO");

            if (result.getBotpose() != null) {
                double x = result.getBotpose().getPosition().x;
                double y = result.getBotpose().getPosition().y;
                double z = result.getBotpose().getPosition().z;
                telemetry.addData("BotPose (m)", "(%.2f, %.2f, %.2f)", x, y, z);
            }

        } else {
            telemetry.addLine("Limelight: sem alvos no frame atual");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}

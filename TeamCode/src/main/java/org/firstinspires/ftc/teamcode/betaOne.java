package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="betaOne", group="Linear OpMode")
public class  betaOne extends LinearOpMode {

/*CONTROLES:
  Joystick = move o robô
  LB = liga os intakes e desliga o shooter/ desliga os intakes e lizga o shooter
  RB = executa a ação de atirar as duas bolinhas pegas de uma vez
  B = rotaciona o robô em direção ao april tag
  Y = distancia o robô para a posição em que sempre acertará (ISSO VAI DEIXAR DE EXISTIR
  POIS SERÁ TROCADO PELO CÁLCULO DE FORÇA DO MOTOR)

  obs programador: trocar os nomes do motores no codigo e dos intakes e shooter no driver hub
 */
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private DcMotor intake1, intake2, shooter;

    private Servo intake3;

    boolean j = false;

    private Limelight3A limelight;

    private IMU imu;

    private boolean movingToTarget = false;
    boolean running = false;

    private double distance;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight = hardwareMap.get(DcMotor.class, "back_right_motor");

        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        double i = 0;

        intake3 = hardwareMap.get(Servo.class, "intake3");

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        intake1.setPower(0);
        shooter.setPower(0);

        intake3.setPosition(0.5);

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientation));

        limelight.pipelineSwitch(0);

        waitForStart();

        limelight.start();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

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

                //ROTACIONADOR AUTOMATICO LIMELIGHT3A
                if (result != null && result.isValid() && gamepad1.b) {
                    double tx = result.getTx();

                    double Kp = 0.035;
                    double error = tx;
                    double turnPower = Kp * error;

                    turnPower = Math.max(Math.min(turnPower, 0.3), -0.3);
                    if (Math.abs(error) < 1.0) {
                        turnPower = 0;
                    }
                    frontLeft.setPower(turnPower);
                    backLeft.setPower(turnPower);
                    frontRight.setPower(-turnPower);
                    backRight.setPower(-turnPower);
                } else {
                    frontLeft.setPower(0);
                    backLeft.setPower(0);
                    frontRight.setPower(0);
                    backRight.setPower(0);
                }

            //CORRETOR DE DISTANCIA LIMELIGHT3A
            double targetOffsetAngle_Vertical = result.getTy();

            double limelightMountAngleDegrees = 0.0;

            double limelightLensHeightInches = 5/2.54;

            double goalHeightInches = 25/2.54;

            double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

            //calculate distance
            distance = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians) * 2.54;

            double KpDistance = -0.1;
            double desiredDistance = 100.0;

            double currentDistance = distance;

            if (gamepad1.a && !movingToTarget) {
                movingToTarget = true;
            }

            if (movingToTarget) {
                double Kp = 0.02;
                double Ke = 0.04;
                double minPower = 0.08;

                double distanceError = desiredDistance - distance;

                double drivingAdjust = Kp * distanceError * Math.exp(-Ke * Math.abs(distanceError));
                drivingAdjust += Math.signum(distanceError) * minPower;

                drivingAdjust = Math.max(Math.min(drivingAdjust, 0.4), -0.4);

                if (Math.abs(distanceError) < 5) {
                    movingToTarget = false;
                    drivingAdjust = 0;
                }

                frontLeft.setPower(drivingAdjust);
                backLeft.setPower(drivingAdjust);
                frontRight.setPower(drivingAdjust);
                backRight.setPower(drivingAdjust);
            } else {
                frontLeft.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
            }

            //CODIGO ALTERNADOR ENTRE INTAKES E SHOOTER
            if (gamepad1.left_bumper && !j) {
                i = i+1;
            }
            if ((i != 0) && (i % 2 == 1)) {
                intake1.setPower(-1.0);
                intake2.setPower(-1.0);
                shooter.setPower(0);
            } else {
                shooter.setPower(-1.0);
                intake1.setPower(0);
                intake2.setPower(0);
            }

                //CODIGO ATIRADOR DO SHOOTER
                if (gamepad1.right_bumper && !running) {
                    running = true;
                    timer.reset();
                }
                if (running && timer.seconds() < 8) {
                    intake1.setPower(-1.0);
                    intake2.setPower(-1.0);
                    intake3.setPosition(1);
                }
                if (running && timer.seconds() >= 8) {
                    running = false;
                    intake1.setPower(0);
                    intake2.setPower(0);
                    intake3.setPosition(0);
                }
            }
        }
    }

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "pidShooter2p")
public class pidShooter2p extends LinearOpMode {

    private DcMotor leftRear;
    private DcMotor leftFront;
    private DcMotor rightRear;
    private DcMotor rightFront;
    private DcMotorEx shooter;
    private DcMotor intake;
    private DcMotor shooterIntake;
    private CRServo intakeServoL;
    private CRServo intakeServoR;
    private Servo blocker;
    private boolean blockerOpen;

    @Override
    public void runOpMode() {
        ElapsedTime runtime;
        float axial;
        float lateral;
        float yaw;
        float frontLeftPower;
        float frontRightPower;
        float backLeftPower;
        float backRightPower;
        double max;

        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooterIntake = hardwareMap.get(DcMotor.class, "shooterIntake");
        intakeServoL = hardwareMap.get(CRServo.class, "intakeServoL");
        intakeServoR = hardwareMap.get(CRServo.class, "intakeServoR");
        blocker = hardwareMap.get(Servo.class, "blocker");
        blocker.setDirection(Servo.Direction.REVERSE);
        blocker.setPosition(0);
        blockerOpen = false;

        initMotors();
        runtime = new ElapsedTime();

        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (gamepad2.xWasPressed()) {
                    shooter.setVelocity(1100);
                } else if (gamepad2.aWasPressed()) {
                    shooter.setVelocity(0);
                }

                if (gamepad2.right_trigger > 0.1) {
                    blocker.setPosition(0);
                    blockerOpen = false;
                } else if (gamepad2.rightBumperWasPressed()) {
                    blocker.setPosition(0.5);
                    blockerOpen = true;
                }

                if (gamepad2.y) {
                    intake.setPower(-1);
                    intakeServoL.setPower(1);
                    intakeServoR.setPower(-1);
                } else if (gamepad2.b) {
                    intake.setPower(0);
                    intakeServoL.setPower(0);
                    intakeServoR.setPower(0);
                }

                if (gamepad2.left_bumper) {
                    shooterIntake.setPower(0.5);
                } else if (gamepad2.leftTriggerWasPressed()) {
                    shooterIntake.setPower(0);
                }

                telemetry.addData("Shooter Velocity", shooter.getVelocity());
                telemetry.addData("Blocker", blockerOpen ? "open" : "closed");

                axial = -gamepad1.left_stick_y;
                lateral = -gamepad1.left_stick_x;
                yaw = gamepad1.right_stick_x;
                frontLeftPower = axial + lateral + yaw;
                frontRightPower = (axial - lateral) - yaw;
                backLeftPower = (axial - lateral) + yaw;
                backRightPower = (axial + lateral) - yaw;
                max = JavaUtil.maxOfList(JavaUtil.createListWith(
                        Math.abs(frontLeftPower),
                        Math.abs(frontRightPower),
                        Math.abs(backLeftPower),
                        Math.abs(backRightPower)
                ));
                if (max > 1) {
                    frontLeftPower = (float) (frontLeftPower / max);
                    frontRightPower = (float) (frontRightPower / max);
                    backLeftPower = (float) (backLeftPower / max);
                    backRightPower = (float) (backRightPower / max);
                }

                leftRear.setPower(frontLeftPower);
                rightRear.setPower(frontRightPower);
                leftFront.setPower(backLeftPower);
                rightFront.setPower(backRightPower);
                telemetry.addData("Status", "Run Time: " + runtime);
                telemetry.addData("Front left/Right",
                        JavaUtil.formatNumber(frontLeftPower, 4, 2) + ", "
                                + JavaUtil.formatNumber(frontRightPower, 4, 2));
                telemetry.addData("Back left/Right",
                        JavaUtil.formatNumber(backLeftPower, 4, 2) + ", "
                                + JavaUtil.formatNumber(backRightPower, 4, 2));
                telemetry.update();
            }
        }
        shooterIntake.setPower(0);
    }

    private void initMotors() {
        shooter.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(300, 0, 0, 10);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setVelocity(0);
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setPower(0);
        shooterIntake.setPower(0);
    }
}

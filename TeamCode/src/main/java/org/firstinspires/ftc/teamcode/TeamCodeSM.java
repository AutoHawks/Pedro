
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeamCodeSM (Fixed)")
public class TeamCodeSM extends LinearOpMode {

    private DcMotor leftRear;
    private DcMotor leftFront;
    private DcMotor rightRear;
    private DcMotor rightFront;

    private DcMotorEx shooter;
    private DcMotor intake;

    private CRServo intakeServoL;
    private CRServo intakeServoR;
    private CRServo shooterServo;
    private CRServo shooterServo2;

    private ElapsedTime launchTime;
    private ElapsedTime shotTimer;
    private ElapsedTime feederTimer;

    private static final double LAUNCHER_TARGET_VELOCITY = 1125;
    private static final double LAUNCHER_MIN_VELOCITY = 1075;

    private static final double TIME_BETWEEN_SHOTS = 2.0;
    private static final double FEEDER_TIME = 2;
    private static final double SPINUP_TIMEOUT = 5.0;

    private static final int SHOTS_TO_FIRE = 4;

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
        WAIT_BETWEEN_SHOTS
    }

    private LaunchState launchState = LaunchState.IDLE;

    private int shotsRemaining = SHOTS_TO_FIRE;

    @Override
    public void runOpMode() {

        // Hardware initialization

        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");

        intakeServoL = hardwareMap.get(CRServo.class, "intakeServoL");
        intakeServoR = hardwareMap.get(CRServo.class, "intakeServoR");

        shooterServo = hardwareMap.get(CRServo.class, "shooterServo");
        shooterServo2 = hardwareMap.get(CRServo.class, "shooterServo2");

        // Initialize timers

        launchTime = new ElapsedTime();
        shotTimer = new ElapsedTime();
        feederTimer = new ElapsedTime();

        // Drivetrain directions

        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        rightRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        // Motor braking

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setVelocityPIDFCoefficients(300, 0, 0, 10);

        // Set all mechanisms to zero

        stopShooter();
        stopIntake();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        launchTime.reset();

        try {

            while (opModeIsActive()) {

                // --------------------------------
                // DRIVETRAIN
                // --------------------------------

                double axial = -gamepad1.left_stick_y;
                double lateral = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x;

                double frontLeftPower = axial + lateral + yaw;
                double frontRightPower = axial - lateral - yaw;
                double backLeftPower = axial - lateral + yaw;
                double backRightPower = axial + lateral - yaw;

                double max = Math.max(
                        1.0,
                        Math.max(
                                Math.max(
                                        Math.abs(frontLeftPower),
                                        Math.abs(frontRightPower)
                                ),
                                Math.max(
                                        Math.abs(backLeftPower),
                                        Math.abs(backRightPower)
                                )
                        )
                );

                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;

                leftFront.setPower(frontLeftPower);
                rightFront.setPower(frontRightPower);
                leftRear.setPower(backLeftPower);
                rightRear.setPower(backRightPower);

                // --------------------------------
                // INTAKE CONTROLS
                // --------------------------------

                if (gamepad1.y) {

                    intake.setPower(-1);

                    intakeServoL.setPower(1);
                    intakeServoR.setPower(-1);

                }
                if (gamepad1.b) {

                    stopIntake();

                }

                // --------------------------------
                // SHOOTER CONTROLS
                // --------------------------------

                if (gamepad1.right_bumper) {
                    // Start shooting when the bumper is pressed.
                    // The state machine prevents restarting while
                    // the current sequence is running.
                }

                if (gamepad1.rightBumperWasPressed()) {

                    if (launchState == LaunchState.IDLE) {

                        shotsRemaining = SHOTS_TO_FIRE;

                        launchState = LaunchState.SPIN_UP;

                        shotTimer.reset();
                        launchTime.reset();
                    }
                }

                updateShooter();

                // --------------------------------
                // TELEMETRY
                // --------------------------------

                telemetry.addData("Status", "Running");
                telemetry.addData("Runtime", launchTime.seconds());

                telemetry.addData(
                        "Shooter Velocity",
                        shooter.getVelocity()
                );

                telemetry.addData(
                        "Shooter State",
                        launchState
                );

                telemetry.addData(
                        "Shots Remaining",
                        shotsRemaining
                );

                telemetry.addData(
                        "Front Left / Right",
                        "%.2f / %.2f",
                        frontLeftPower,
                        frontRightPower
                );

                telemetry.addData(
                        "Back Left / Right",
                        "%.2f / %.2f",
                        backLeftPower,
                        backRightPower
                );

                telemetry.update();
            }

        } finally {

            // Stop everything when OpMode ends

            stopShooter();
            stopIntake();

            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
        }
    }

    // --------------------------------
    // SHOOTER STATE MACHINE
    // --------------------------------

    private void updateShooter() {

        switch (launchState) {

            case IDLE:

                stopShooterServos();

                break;

            case SPIN_UP:

                shooter.setVelocity(LAUNCHER_TARGET_VELOCITY);

                if (shooter.getVelocity() >= LAUNCHER_MIN_VELOCITY) {

                    launchState = LaunchState.LAUNCH;

                } else if (shotTimer.seconds() > SPINUP_TIMEOUT) {

                    stopShooter();

                    launchState = LaunchState.IDLE;

                    telemetry.addData(
                            "Shooter Warning",
                            "Flywheel failed to reach target speed"
                    );
                }

                break;

            case LAUNCH:

                shooterServo.setPower(-1);
                shooterServo2.setPower(1);

                feederTimer.reset();

                launchState = LaunchState.LAUNCHING;

                break;

            case LAUNCHING:

                if (feederTimer.seconds() >= FEEDER_TIME) {

                    stopShooterServos();

                    shotsRemaining--;

                    if (shotsRemaining > 0) {

                        shotTimer.reset();

                        launchState = LaunchState.WAIT_BETWEEN_SHOTS;

                    } else {

                        stopShooter();

                        launchState = LaunchState.IDLE;
                    }
                }

                break;

            case WAIT_BETWEEN_SHOTS:

                shooter.setVelocity(LAUNCHER_TARGET_VELOCITY);

                if (shotTimer.seconds() >= TIME_BETWEEN_SHOTS) {

                    launchState = LaunchState.SPIN_UP;

                    shotTimer.reset();
                }

                break;
        }
    }

    // --------------------------------
    // INTAKE FUNCTIONS
    // --------------------------------

    private void stopIntake() {

        intake.setPower(0);

        intakeServoL.setPower(0);
        intakeServoR.setPower(0);
    }

    // --------------------------------
    // SHOOTER FUNCTIONS
    // --------------------------------

    private void stopShooterServos() {

        shooterServo.setPower(0);
        shooterServo2.setPower(0);
    }

    private void stopShooter() {

        shooter.setVelocity(0);

        stopShooterServos();
    }

}

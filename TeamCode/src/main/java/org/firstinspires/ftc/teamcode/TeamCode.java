package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "TeamCode (Blocks to Java)")
public class TeamCode extends LinearOpMode {

    private DcMotor leftRear;
    private DcMotor leftFront;
    private DcMotor rightRear;
    private DcMotor rightFront;
    private DcMotor shooter;
    private DcMotor intake;
    private CRServo intakeServoL;
    private CRServo intakeServoR;
    private CRServo shooterServo;
    private CRServo shooterServo2;

    ElapsedTime launchTime;
    String LAUNCH_BALLS;
    String IDLE;
    int LAUNCHER_TARGET_VELOCITY;
    String WAIT_FOR_LAUNCH;
    String SPIN_UP;
    int LAUNCHER_MIN_VELOCITY;
    String DRIVING_AWAY_FROM_GOAL;
    String LAUNCH;
    String launchState;
    String LAUNCHING;
    int TIME_BETWEEN_SHOTS;
    ElapsedTime shotTimer;
    ElapsedTime FeederTimer;
    String autoStateMachine;
    int ShotsToFire;

    /**
     * This OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
     * This code will work with either a Mecanum-Drive or an X-Drive train.
     * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
     *
     * Also note that it is critical to set the correct rotation direction for each motor. See details below.
     *
     * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
     * Each motion axis is controlled by one Joystick axis.
     *
     * 1) Axial -- Driving forward and backward -- Left-joystick Forward/Backward
     * 2) Lateral -- Strafing right and left -- Left-joystick Right and Left
     * 3) Yaw -- Rotating Clockwise and counter clockwise -- Right-joystick Right and Left
     *
     * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
     * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
     * the direction of all 4 motors (see code below).
     */
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
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeServoL = hardwareMap.get(CRServo.class, "intakeServoL");
        intakeServoR = hardwareMap.get(CRServo.class, "intakeServoR");
        shooterServo = hardwareMap.get(CRServo.class, "shooterServo");
        shooterServo2 = hardwareMap.get(CRServo.class, "shooterServo2");

        createAutoStateMachine();
        createLaunchStateMachine();
        createVariables();
        intiMotors();
        runtime = new ElapsedTime();
        // ########################################################################################
        // !!! IMPORTANT Drive Information. Test your motor directions. !!!!!
        // ########################################################################################
        //
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot
        // (the wheels turn the same direction as the motor shaft).
        //
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction. So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        //
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward.
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        // <--- Click blue icon to see important note re. testing motor directions.
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        // Run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (autoStateMachine.equals(LAUNCH_BALLS)) {
                    launch(true);
                    autoStateMachine = WAIT_FOR_LAUNCH;
                } else if (autoStateMachine.equals(WAIT_FOR_LAUNCH)) {
                    if (launch(false)) {
                        ShotsToFire += -1;
                        if (ShotsToFire > 0) {
                            autoStateMachine = LAUNCH_BALLS;
                        } else {
                            ((DcMotorEx) shooter).setVelocity(0);
                            autoStateMachine = DRIVING_AWAY_FROM_GOAL;
                        }
                    }
                }
                if (gamepad1.y) {
                    intake.setPower(-1);
                    intakeServoL.setPower(1);
                    intakeServoR.setPower(-1);
                } else if (gamepad1.b) {
                    intake.setPower(0);
                    intakeServoL.setPower(0);
                    intakeServoR.setPower(0);
                } else if (gamepad1.x) {
                    ((DcMotorEx) shooter).setVelocity(1300);
                    shooterServo.setPower(-1);
                    shooterServo2.setPower(1);
                } else if (gamepad1.a) {
                    ((DcMotorEx) shooter).setVelocity(0);
                    shooterServo.setPower(0);
                    shooterServo2.setPower(0);
                }
                telemetry.addData("Launch State", launchState);
                telemetry.addData("Launcher Motor Velocity", ((DcMotorEx) shooter).getVelocity());
                telemetry.addData("Launch time", launchTime);
                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                // Note: pushing stick forward gives negative value
                axial = -gamepad1.left_stick_y;
                lateral = -gamepad1.left_stick_x;
                yaw = gamepad1.right_stick_x;
                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                // Set up a variable for each drive wheel to save the power level for telemetry.
                frontLeftPower = axial + lateral + yaw;
                frontRightPower = (axial - lateral) - yaw;
                backLeftPower = (axial - lateral) + yaw;
                backRightPower = (axial + lateral) - yaw;
                // Normalize the values so no wheel power exceeds 100%
                // This ensures that the robot maintains the desired motion.
                max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(frontLeftPower), Math.abs(frontRightPower), Math.abs(backLeftPower), Math.abs(backRightPower)));
                if (max > 1) {
                    frontLeftPower = (float) (frontLeftPower / max);
                    frontRightPower = (float) (frontRightPower / max);
                    backLeftPower = (float) (backLeftPower / max);
                    backRightPower = (float) (backRightPower / max);
                }
                // Send calculated power to wheels.
                leftRear.setPower(frontLeftPower);
                rightRear.setPower(frontRightPower);
                leftFront.setPower(backLeftPower);
                rightFront.setPower(backRightPower);
                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime);
                telemetry.addData("Front left/Right", JavaUtil.formatNumber(frontLeftPower, 4, 2) + ", " + JavaUtil.formatNumber(frontRightPower, 4, 2));
                telemetry.addData("Back  left/Right", JavaUtil.formatNumber(backLeftPower, 4, 2) + ", " + JavaUtil.formatNumber(backRightPower, 4, 2));
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void intiMotors() {
        shooter.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setPower(0);
        ((DcMotorEx) shooter).setVelocityPIDFCoefficients(300, 0, 0, 10);
    }

    /**
     * Describe this function...
     */
    private void createVariables() {
        double DRIVE_SPEED;
        double ROTATE_SPEED;
        int WHEEL_DIAMETER_MM;
        double ENCODER_TICKS_PER_REV;
        double TICKS_PER_INCH;
        double TRACK_WIDTH_INCHES;
        int robotRotateAngle;
        double DRIVE_TOLERANCE;
        int holdSeconds;
        ElapsedTime driveTimer;

        launchTime = new ElapsedTime();
        LAUNCHER_TARGET_VELOCITY = 1125;
        LAUNCHER_MIN_VELOCITY = 1075;
        TIME_BETWEEN_SHOTS = 2;
        DRIVE_SPEED = 0.5;
        ROTATE_SPEED = 0.2;
        WHEEL_DIAMETER_MM = 96;
        ENCODER_TICKS_PER_REV = 537.7;
        TICKS_PER_INCH = ENCODER_TICKS_PER_REV / ((WHEEL_DIAMETER_MM * Math.PI) / 25.4);
        TRACK_WIDTH_INCHES = 15.9;
        ShotsToFire = 3;
        robotRotateAngle = 45;
        DRIVE_TOLERANCE = 0.5;
        holdSeconds = 1;
        shotTimer = new ElapsedTime();
        FeederTimer = new ElapsedTime();
        driveTimer = new ElapsedTime();
    }

    /**
     * Describe this function...
     */
    private void createAutoStateMachine() {
        String ROTATING;
        String DRIVING_OFF_LINE;
        String COMPLETE;

        LAUNCH_BALLS = "LAUNCH_BALLS";
        WAIT_FOR_LAUNCH = "WAIT_FOR_LAUNCH";
        DRIVING_AWAY_FROM_GOAL = "DRIVING_AWAY_FROM_GOAL";
        ROTATING = "ROTATING";
        DRIVING_OFF_LINE = "DRIVING_OFF_LINE";
        COMPLETE = "COMPLETE";
        autoStateMachine = LAUNCH_BALLS;
    }

    /**
     * Describe this function...
     */
    private void createLaunchStateMachine() {
        IDLE = "IDLE";
        SPIN_UP = "SPIN_UP";
        LAUNCH = "LAUNCH";
        LAUNCHING = "LAUNCHING";
        launchState = IDLE;
    }

    /**
     * Describe this function...
     */
    private void arcadeDrive(double forward, double rotate) {
        leftFront.setPower(forward + rotate);
        leftRear.setPower(forward + rotate);
        rightFront.setPower(forward + rotate);
        rightRear.setPower(forward + rotate);
    }

    /**
     * Describe this function...
     */
    private boolean launch(boolean shotRequested) {
        if (launchState.equals(IDLE)) {
            if (shotRequested == true) {
                launchState = SPIN_UP;
                shotTimer.reset();
            }
        } else if (launchState.equals(SPIN_UP)) {
            ((DcMotorEx) shooter).setVelocity(LAUNCHER_TARGET_VELOCITY);
            if (((DcMotorEx) shooter).getVelocity() > LAUNCHER_MIN_VELOCITY) {
                launchState = LAUNCH;
            }
        } else if (launchState.equals(LAUNCH)) {
            shooterServo.setPower(1);
            shooterServo2.setPower(1);
            FeederTimer.reset();
            launchState = LAUNCHING;
        } else if (launchState.equals(LAUNCHING)) {
            if (FeederTimer.seconds() > 0.2) {
                shooterServo.setPower(0);
                shooterServo.setPower(0);
                if (shotTimer.seconds() > TIME_BETWEEN_SHOTS) {
                    launchState = IDLE;
                    if (true) {
                        return true;
                    }
                }
            }
        }
        return false;
    }
}
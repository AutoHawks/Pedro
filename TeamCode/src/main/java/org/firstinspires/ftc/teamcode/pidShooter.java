package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "pidShooter")
public class pidShooter extends LinearOpMode {

    private DcMotor leftRear;
    private DcMotor leftFront;
    private DcMotor rightRear;
    private DcMotor rightFront;
    private DcMotorEx shooter;
    private DcMotor intake;
    private DcMotor shooterIntake;
    private CRServo intakeServoL;
    private CRServo intakeServoR;

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
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooterIntake = hardwareMap.get(DcMotor.class, "shooterIntake");
        intakeServoL = hardwareMap.get(CRServo.class, "intakeServoL");
        intakeServoR = hardwareMap.get(CRServo.class, "intakeServoR");

        initMotors();
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
                if (gamepad1.xWasPressed()) {
                    shooter.setVelocity(1100);
                } else if (gamepad1.aWasPressed()) {
                    shooter.setPower(0);
                }
                if (gamepad1.y) {
                    intake.setPower(-1);
                    intakeServoL.setPower(1);
                    intakeServoR.setPower(-1);
                } else if (gamepad1.b) {
                    intake.setPower(0);
                    intakeServoL.setPower(0);
                    intakeServoR.setPower(0);
                }
                if (gamepad1.left_bumper) {
                    shooterIntake.setPower(0.5);
                } else if (gamepad1.leftTriggerWasPressed()) {
                    shooterIntake.setPower(0);
                }
                telemetry.addData("Shooter Velocity", shooter.getVelocity());
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
        shooterIntake.setPower(0);
    }

    /**
     * Describe this function...
     */
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
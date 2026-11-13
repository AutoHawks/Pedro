package org.firstinspires.ftc.teamcode;

// Pedro pathing library imports used to define robot motion and autonomous commands.

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

// Static imports make the autonomous scheduling calls easier to read.

// This OpMode drives the robot through a blue-side, high-side autonomous path.
@Autonomous(name = "highSideLeftRed", group = "Autonomous")
public class highSideLeftRed extends LinearOpMode {

    // The robot follower object handles movement and tracking during autonomous execution.
    private Follower follower;
    private DcMotorEx shooter;
    private CRServo shooterServo;
    private CRServo shooterServo2;
    private DcMotorEx shooterIntake;
    private final ElapsedTime shootingDelayTimer = new ElapsedTime();
    private boolean feederRunning;

    // Same shooting settings and variable names as the PathingTest routine.
    private static final long SHOOTING_DELAY_MS = 10000;
    private static final double LAUNCHER_TARGET_VELOCITY = 1125;
    private static final double LAUNCHER_MIN_VELOCITY = 1075;
    // Intake feeds balls into the shooter. Flip to -1 if it spins the wrong way.
    private static final double INTAKE_FEED_POWER = 0.5;

    // The PoseFactory creates field positions using degrees for heading values.
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Waypoints used in the autonomous route. Each point is a field position (x, y, heading).
    // The robot starts near the high side and moves through a series of points to reach the target zone.
    private final Pose start = poseFactory.of(57.75, 8.75, 270);
    private final Pose path1 = poseFactory.of(57.75, 21.75, 180);
    private final Pose point2Start = poseFactory.of(57.75, 21.75, 0);
    private final Pose point2 = poseFactory.of(32.75, 21.75, 0);
    private final Pose point3Start = poseFactory.of(32.75, 21.75, 0);
    private final Pose point3 = poseFactory.of(21.75, 76.75, 0);

    // Builds the full autonomous routine by chaining several path-following commands in order.
    private Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3())
        );
    }

    @Override
    public void runOpMode() {
        // Reset the scheduler before starting a new autonomous routine.
        Scheduler.reset();

        // Create a Pedro follower using the robot's drivetrain and sensor configuration.
        follower = Constants.create(hardwareMap);
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterServo = hardwareMap.get(CRServo.class, "shooterServo");
        shooterServo2 = hardwareMap.get(CRServo.class, "shooterServo2");
        shooterIntake = hardwareMap.get(DcMotorEx.class, "shooterIntake");

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setVelocityPIDFCoefficients(300, 0, 0, 10);
        stopShooter();

        // Place the robot at the starting coordinate before the match begins.
        follower.setPose(start);
        follower.update();

        // Tell the driver station the robot is ready and wait for the start signal.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // If the op mode is canceled before activation, stop immediately.
        if (!opModeIsActive()) {
            return;
        }

        // Shoot at the start, then run the path sequence.
        shootPreload();
        if (!opModeIsActive()) {
            stopShooter();
            return;
        }

        // Schedule the move sequence and then keep updating the follower and scheduler during the match.
        schedule(autoRoutine());
        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            // Report current robot position and heading for debugging and tuning.
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            if (follower.currentPath() != null) {
                telemetry.addData("Path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }
            telemetry.update();
        }
        stopShooter();
    }

    // Spins up the shooter, then once it is at speed runs the feeder servos and shooterIntake
    // continuously until the shooting delay is over. The delay is counted from the start of
    // spin-up, same as the PathingTest routine.
    private void shootPreload() {
        try {
            shootingDelayTimer.reset();
            feederRunning = false;

            while (opModeIsActive() && shootingDelayTimer.milliseconds() < SHOOTING_DELAY_MS) {
                shooter.setVelocity(LAUNCHER_TARGET_VELOCITY);

                if (!feederRunning && shooter.getVelocity() >= LAUNCHER_MIN_VELOCITY) {
                    shooterServo.setPower(-1);
                    shooterServo2.setPower(1);
                    shooterIntake.setPower(INTAKE_FEED_POWER);
                    feederRunning = true;
                }

                telemetry.addData("Shooting", feederRunning ? "Feeding" : "Spinning up");
                telemetry.addData("Shooter velocity", shooter.getVelocity());
                telemetry.update();
                idle();
            }
        } finally {
            stopShooter();
        }
    }

    private void stopShooter() {
        shooter.setVelocity(0);
        shooterServo.setPower(0);
        shooterServo2.setPower(0);
        shooterIntake.setPower(0);
        feederRunning = false;
    }

    // Path 1: move slightly away from the starting point while aligning with the intended direction.
    private Path path1() {
        return Paths.line(start, path1).reverseTangent();
    }

    // Path 2: travel forward along the high side while keeping a constant heading.
    private Path path2() {
        return Paths.line(path1, point2).constant(90);
    }

    // Path 3: continue toward the target area and rotate into the next heading as needed.
    private Path path3() {
        return Paths.line(point2, point3).tangent();
    }

}
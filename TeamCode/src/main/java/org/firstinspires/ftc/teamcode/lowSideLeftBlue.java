package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "lowSideLeftBlue", group = "Autonomous")
public class lowSideLeftBlue extends LinearOpMode {
    private Follower follower;
    private DcMotorEx shooter;
    private CRServo shooterServo;
    private CRServo shooterServo2;
    private DcMotorEx shooterIntake;
    private final ElapsedTime shootingDelayTimer = new ElapsedTime();
    private boolean feederRunning;

    // Same shooting settings and variable names as the other routines.
    // Delay at the start of auto (milliseconds). 5000 = 5 seconds.
    private static final long START_DELAY_MS = 8000;
    private static final long SHOOTING_DELAY_MS = 10000;
    private static final double LAUNCHER_TARGET_VELOCITY = 1125;
    private static final double LAUNCHER_MIN_VELOCITY = 1075;
    // Intake feeds balls into the shooter. Flip to -1 if it spins the wrong way.
    private static final double INTAKE_FEED_POWER = 0.5;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(81.5255, 11.6993, 270);
    private final Pose path1 = poseFactory.of(80.6007, 23.9771, 180);
    private final Pose point2 = poseFactory.of(137.0387, 23.4981, 179.4568);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
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

        follower.setPose(start);
        follower.update();

        waitForStart();

        if (!opModeIsActive()) {
            return;
        }

        sleep(START_DELAY_MS);
        if (!opModeIsActive()) {
            return;
        }

        // Shoot at the start, then run the path sequence.
        shootPreload();
        if (!opModeIsActive()) {
            stopShooter();
            return;
        }

        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }

        stopShooter();
    }

    // Spins up the shooter, then once it is at speed runs the feeder servos and shooterIntake
    // continuously until the shooting delay is over. The delay is counted from the start of
    // spin-up.
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

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(path1, point2).reverseTangent();
    }
}
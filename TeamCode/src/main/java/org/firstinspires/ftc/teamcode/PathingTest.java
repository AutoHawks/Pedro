package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class PathingTest extends LinearOpMode {

    private Follower follower;
    private DcMotorEx shooter;
    private DcMotor shooterIntake;
    private CRServo shooterServo;
    private CRServo shooterServo2;
    private final ElapsedTime shootingDelayTimer = new ElapsedTime();
    private boolean shootingDuringDelay;
    private boolean feederRunning;

    private static final long SHOOTING_DELAY_MS = 10000;
    private static final double LAUNCHER_TARGET_VELOCITY = 1100;
    private static final double LAUNCHER_MIN_VELOCITY = 1075;
    // Intake feeds balls into the shooter. Flip to -1 if it spins the wrong way.
    private static final double INTAKE_FEED_POWER = 0.5;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses
    private final Pose start = poseFactory.of(106.75, 10.75, 90);
    private final Pose path1End = poseFactory.of(106.75, 130.75, 90);
    // Shooting pose: back of robot faces the goal.
    // Change this angle if your goal or shooter side is different.
    private final Pose point2 = poseFactory.of(82.75, 130.75, 92);
    private final Pose point3 = poseFactory.of(106.75, 130.75, 0);
    // Moved up from y=22.75 to leave room to brake before the bottom wall.
    // Heading 0 so the robot strafes down and doesn't have to turn before parking.
    private final Pose point4 = poseFactory.of(106.75, 36, 0);
    // Moved in from x=130.75 so the robot body doesn't hit the right wall.
    private final Pose point5 = poseFactory.of(122, 36, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                hold(follower, point2),
                shootingDelay(),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                hold(follower, point5)
        );
    }

    private Command shootingDelay() {
        return new CommandBuilder()
                .setStart(() -> {
                    shootingDelayTimer.reset();
                    shootingDuringDelay = true;
                    feederRunning = false;
                })
                .setDone(() -> {
                    if (shootingDelayTimer.milliseconds() < SHOOTING_DELAY_MS) {
                        return false;
                    }

                    shootingDuringDelay = false;
                    stopShooter();
                    return true;
                });
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterServo = hardwareMap.get(CRServo.class, "shooterServo");
        shooterServo2 = hardwareMap.get(CRServo.class, "shooterServo2");
        shooterIntake = hardwareMap.get(DcMotor.class, "shooterIntake");

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setVelocityPIDFCoefficients(300, 0, 0, 10);

        shooterIntake.setDirection(DcMotor.Direction.FORWARD);
        shooterIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stopShooter();

        follower.setPose(start);
        follower.update();

        // Optional: slow the whole auto down if it still overshoots.
        // Uncomment if this method exists in your Pedro version.
        // follower.setMaxPower(0.6);

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            updateShooter();
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

    private void updateShooter() {
        if (!shootingDuringDelay) {
            return;
        }

        shooter.setVelocity(LAUNCHER_TARGET_VELOCITY);
        if (!feederRunning && shooter.getVelocity() >= LAUNCHER_MIN_VELOCITY) {
            shooterServo.setPower(-1);
            shooterServo2.setPower(1);
            shooterIntake.setPower(INTAKE_FEED_POWER);
            feederRunning = true;
        }
    }

    private void stopShooter() {
        shooter.setVelocity(0);
        shooterServo.setPower(0);
        shooterServo2.setPower(0);
        shooterIntake.setPower(0);
        feederRunning = false;
    }

    // Drive up the right side, front leading
    public Path path1() {
        return Paths.line(start, path1End).tangent();
    }

    // Drive left and rotate from path1End's heading to the shooting heading (point2)
    public Path path2() {
        return Paths.line(path1End, point2)
                .heading(Interpolator.piecewise().until(1, Interpolator.linear(path1End, point2)));
    }

    // Heading doesn't matter after shooting, so just follow the direction of travel
    public Path path3() {
        return Paths.line(point2, point3).tangent();
    }

    // Keep heading fixed (0 degrees) so the robot strafes down without turning
    public Path path4() {
        return Paths.line(point3, point4).constant(point4);
    }

    // Keep heading fixed while driving to the park spot
    public Path path5() {
        return Paths.line(point4, point5).constant(point4);
    }
}
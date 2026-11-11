package org.firstinspires.ftc.teamcode;

// Pedro pathing library imports used to define robot motion and autonomous commands.
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;

// Static imports make the autonomous scheduling calls easier to read.
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

// This OpMode drives the robot through a blue-side, high-side autonomous path.
@Autonomous(name = "highSideLeftBlue", group = "Autonomous")
public class highSideLeftBlue extends LinearOpMode {

    // The robot follower object handles movement and tracking during autonomous execution.
    private Follower follower;

    // The PoseFactory creates field positions using degrees for heading values.
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Waypoints used in the autonomous route. Each point is a field position (x, y, heading).
    // The robot starts near the high side and moves through a series of points to reach the target zone.
    private final Pose start = poseFactory.of(82.75, 129.75, 90);
    private final Pose point1 = poseFactory.of(82.75, 128.75, 90);
    private final Pose point2 = poseFactory.of(115.75, 108.75, 90);
    private final Pose point3 = poseFactory.of(115.75, 30.75, -90);
    private final Pose point4 = poseFactory.of(130.75, 30.75, 180);

    // Builds the full autonomous routine by chaining several path-following commands in order.
    private Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4())
        );
    }

    @Override
    public void runOpMode() {
        // Reset the scheduler before starting a new autonomous routine.
        Scheduler.reset();

        // Create a Pedro follower using the robot's drivetrain and sensor configuration.
        follower = Constants.create(hardwareMap);

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
    }

    // Path 1: move slightly away from the starting point while aligning with the intended direction.
    private Path path1() {
        return Paths.line(start, point1).reverseTangent();
    }

    // Path 2: travel forward along the high side while keeping a constant heading.
    private Path path2() {
        return Paths.line(point1, point2).constant(90);
    }

    // Path 3: continue toward the target area and rotate into the next heading as needed.
    private Path path3() {
        return Paths.line(point2, point3).tangent();
    }

    // Path 4: finish the route with a final angled movement to the endpoint.
    private Path path4() {
        return Paths.line(point3, point4).reverseTangent();
    }
}

/*
 * ===========================================================================
 * Limelight 3A -- AprilTag targeting, with turn-toward-tag drive
 * ===========================================================================
 *
 * WHAT THIS IS
 *   The AprilTag targeting test, plus one behaviour on top of it: the robot
 *   turns on the spot until the chosen tag sits in the middle of the camera.
 *
 *   Every frame it:
 *     1. reads every AprilTag the Limelight can see,
 *     2. drops the ones the ID gate excludes (see THE APRILTAG ID GATE),
 *     3. converts the rest into LLTarget objects,
 *     4. ORDERS that list with one strategy (see the next section),
 *     5. takes the first entry of the ordered list as "the target",
 *     6. works out the turn command that would put that target in the middle
 *        of the camera,
 *     7. sends that turn to the four drive motors -- but only when the driver
 *        has armed it with Y (see TURNING TOWARD THE TAG).
 *
 *   Step 6 is the same predictor + fine-zone pulse logic used by
 *   "Limelight: Test", so the turn number here is the number that OpMode
 *   drives with.
 *
 *   The camera half still runs on a robot with no drivetrain configured: the
 *   motors are optional, and the screen says so if they are missing.
 *
 * ---------------------------------------------------------------------------
 * TURNING TOWARD THE TAG
 * ---------------------------------------------------------------------------
 *   Turning is OFF when the OpMode starts, so pressing START on a robot with
 *   wheels never surprises anybody. Y toggles it; the "Turn" line shows the
 *   state and the power actually sent.
 *
 *   Only the yaw axis is driven. Axial and lateral stay at zero, so the robot
 *   pivots in place and the camera keeps the tag in view instead of driving
 *   into it. When turning is off, when there is no target, or when the four
 *   motors are not all in the robot configuration, the wheels are set to 0.
 *
 *   Sign: txnc > 0 means the tag is right of centre, and a positive turn means
 *   clockwise, so the targeting maths needs no flip. The single flip is
 *   YAW_SIGN in drive(), exactly as in "Limelight: Test".
 *
 * ---------------------------------------------------------------------------
 * THE THREE ORDERING STRATEGIES  (this is the part you edit)
 * ---------------------------------------------------------------------------
 *   Exactly ONE of the three blocks marked "STRATEGY A / B / C" defines
 *   orderTargets(). The other two are commented out.
 *
 *   To pick a strategy: delete the two blocks you do not want. The block that
 *   is left is then the only definition of orderTargets(), so it is the one
 *   that runs. There is no mode constant to change and no switch to edit.
 *
 *     A  BY SIZE       biggest apparent area first   (nearest / clearest)
 *     B  BY POSITION   smallest |tx| first           (most centred in view)
 *     C  RANDOM        shuffled every frame          (test / sanity check)
 *
 *   Uncomment the one you want BEFORE deleting the one that is active,
 *   otherwise the file stops compiling in between.
 *
 *   Because the caller only ever takes list.get(0), "ordering" and "choosing"
 *   are the same operation. That is why one sort is all it takes.
 *
 * ---------------------------------------------------------------------------
 * THE APRILTAG ID GATE
 * ---------------------------------------------------------------------------
 *   An AprilTag has an ID: the whole number encoded in the tag itself (0, 1,
 *   2, 3, ... depending on the family). A frame can contain several tags at
 *   once, so this OpMode has a gate that decides which IDs are even allowed to
 *   become candidates:
 *
 *       TARGET_ID_GATE = -1   no gate, every tag in view is a candidate
 *       TARGET_ID_GATE =  3   only tag #3 is a candidate
 *
 *   The gate is applied in buildTargets(), BEFORE the ordering strategy runs,
 *   so a strategy never has to know the gate exists. A tag that fails the gate
 *   is dropped exactly like a tag that was never in view.
 *
 *   At runtime the bumpers walk the gate through "any" plus the IDs actually
 *   in view, so you can point at any tag without recompiling:
 *
 *       RB   next ID       (-1 -> #2 -> #5 -> -1 ...)
 *       LB   previous ID
 *
 *   TARGET_ID_GATE is only the STARTUP value. If the gate is set to an ID that
 *   is not in view, the target reads "none" on purpose -- that is the gate
 *   doing its job, not a detection failure.
 *
 * ---------------------------------------------------------------------------
 * WHAT IT PRINTS
 * ---------------------------------------------------------------------------
 *   Limelight   connection state and how the device was found
 *   Camera      pipeline index/type, fps, temperature, CPU
 *   Result      live or held, frame age, capture latency, tag count
 *   Order       which strategy is compiled in
 *   Gate        the active ID gate and every ID currently visible
 *   Turn        armed/disarmed, and the power actually sent to the wheels
 *   Config motors  every DcMotor name the hub actually has (only shown when
 *               the four drive motors could not be found, so a config typo is
 *               visible on the Driver Station instead of guessed at)
 *   Target      the chosen tag: family, id, tx/ty, raw area, range
 *   Aim         error, error rate, predicted error, turn command
 *   List        the whole ordered list, first few entries
 *
 * ---------------------------------------------------------------------------
 * CONTROLS
 * ---------------------------------------------------------------------------
 *   Y .................. arm / disarm turning toward the tag (starts disarmed)
 *   D-pad up / down .... previous / next pipeline (0 - 9)
 *   LB / RB ............ previous / next AprilTag ID gate ("any", then each
 *                        ID currently in view)
 *   B .................. reload the active pipeline
 *   X .................. drop the held target
 *   A .................. restart the Limelight stream
 *
 * ---------------------------------------------------------------------------
 * BEFORE TAGS CAN APPEAR
 * ---------------------------------------------------------------------------
 *   The active pipeline has to be a fiducial / AprilTag pipeline. If the
 *   Limelight is still on a colour pipeline the screen will say
 *   "tags 0  pipe 0 (color)" and that is expected, not a fault.
 *
 *   Importing a ready-made 36h11 pipeline: in the Limelight web UI open
 *   Pipelines, switch to a FREE slot (not the one your colour pipeline lives
 *   in), and drag the .vpr file in.
 *
 * ---------------------------------------------------------------------------
 * REUSING THIS IN A DRIVING OPMODE
 * ---------------------------------------------------------------------------
 *   The parts worth copying are small and self-contained:
 *
 *     buildTargets(LLResult, int) turn the raw SDK result into LLTarget
 *                                 objects; the int is the ID gate (-1 = any)
 *     orderTargets(List)          the strategy you chose
 *     aimTurn(LLTarget, long)     the turn command, in -1..1
 *     drive(axial, lateral, yaw)  the mecanum mixer, with YAW_SIGN in it
 *
 *   Then in your loop:
 *
 *     List<LLTarget> targets = buildTargets(result, -1);
 *     orderTargets(targets);
 *     double turn = targets.isEmpty() ? 0.0 : aimTurn(targets.get(0), now);
 *     drive(axial, lateral, turn);        // turn > 0 = clockwise / right
 *
 *   This file is deliberately not a subclass of anything, so it can be copied
 *   into any project without dragging a base class along.
 *
 * ---------------------------------------------------------------------------
 * PORTABILITY
 * ---------------------------------------------------------------------------
 *   Self-contained: only the FTC SDK and a Limelight 3A are needed. The device
 *   is found by any of the names below, by any Limelight3A in the robot
 *   configuration, or by a direct Ethernet-over-USB connection at 172.29.0.1
 *   when the configuration does not list it yet.
 *
 *   The drive motors are found the same way, by candidate name lists, so the
 *   file moves between robots unchanged as long as one of the listed names
 *   exists in the configuration. A robot with no drive motors still runs the
 *   camera half.
 *
 *   OnBotJava needs the file name and the class name to match, so this file is
 *   "apriltagtargeting.java" with class "apriltagtargeting". In Android Studio
 *   rename both together (AprilTagTargeting.java / AprilTagTargeting).
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.lang.reflect.Constructor;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.InterfaceAddress;
import java.net.NetworkInterface;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Enumeration;
import java.util.List;
import java.util.Random;
import java.util.SortedSet;

@TeleOp(name = "Limelight: AprilTag Targeting", group = "Limelight")
public class apriltagtargeting extends LinearOpMode {

    // ---------------------------------------------------------------------
    // Device names. The first name that exists in the robot configuration
    // wins. Add your own names here; no other change is needed.
    // ---------------------------------------------------------------------
    private static final String[] LIMELIGHT_NAMES = {
            "limelight", "Limelight", "limelight3a", "Limelight3A"
    };
    // Used only for the direct connection, when the configuration does not
    // list the Limelight yet.
    private static final String LIMELIGHT_OBJECT_NAME   = "limelight";
    private static final String LIMELIGHT_IP            = "172.29.0.1";
    private static final String LIMELIGHT_SUBNET_PREFIX = "172.29.";

    // ---------------------------------------------------------------------
    // Drive motor names. Same candidate-list idea as the Limelight: the first
    // name that exists in the robot configuration wins, and the rest are
    // ignored. Both the "leftFront" style and the "frontLeft" style are
    // listed so this file survives being moved between the team's robots.
    // ---------------------------------------------------------------------
    private static final String[] LEFT_FRONT_NAMES  = { "leftFront"};
    private static final String[] LEFT_REAR_NAMES   = { "leftRear"};
    private static final String[] RIGHT_FRONT_NAMES = { "rightFront"};
    private static final String[] RIGHT_REAR_NAMES  = { "rightRear"};

    // ---------------------------------------------------------------------
    // Polling
    // ---------------------------------------------------------------------
    // How often the SDK polls the camera. 30 Hz is plenty: the camera itself
    // runs at ~40 fps, and the SDK hands back the newest cached result.
    private static final int    POLL_RATE_HZ      = 30;
    private static final int    PIPELINE_COUNT    = 10;
    private static final int    START_PIPELINE    = 0;

    // getStatus() is a separate HTTP call, so it is cached.
    private static final long   STATUS_REFRESH_MS = 1000;

    // A tag that drops out for a frame or two should not blank the screen, so
    // the last non-empty list is reused for this long.
    private static final long   HOLD_LAST_MS      = 500;
    // A result older than this counts as "no target" no matter what it says.
    private static final double MAX_RESULT_AGE_MS = 500.0;

    private static final long   LOG_INTERVAL_MS   = 500;
    private static final int    MAX_LISTED        = 4;

    // ---------------------------------------------------------------------
    // AprilTag ID gate
    // ---------------------------------------------------------------------
    // Which tag IDs are allowed to become candidates.
    //
    //   -1   any tag (no gate). This is the default.
    //   >=0  only that exact tag ID. A tag #7 is the number printed on the tag,
    //        not its position in the list.
    //
    // This is only the STARTUP value. The bumpers change it at runtime, so you
    // can point at any tag without recompiling. Setting it to an ID that is not
    // in view is legal: the target simply reads "none" until that tag appears.
    private static final int    TARGET_ID_GATE    = -1;

    // ---------------------------------------------------------------------
    // Aim -- copied from "Limelight: Test" so the numbers match
    // ---------------------------------------------------------------------
    // Aim at the camera CENTRE (txnc), not the crosshair (tx). The crosshair
    // can be dragged off-centre in the Limelight web UI; aiming on tx then
    // parks the target at the edge of the view.
    //
    // Sign convention used everywhere in this file:
    //   txnc > 0  -> the target is to the RIGHT of the camera centre
    //   turn > 0  -> the robot should rotate CLOCKWISE (to its right)
    // The two therefore have the same sign and no flip is applied here.
    // The flip lives in the drivetrain mixer, not in the targeting maths.
    private static final double TX_DEADBAND_DEG    = 1.0;
    private static final double AIM_KP             = 0.016;
    private static final double AIM_MIN_POWER      = 0.07;
    private static final double AIM_MAX_POWER      = 0.22;
    private static final double AIM_SLOW_ZONE_DEG  = 10.0;
    private static final double AIM_SLOW_MAX_POWER = 0.12;

    // Lead / predictor. The camera is slow compared with the control loop, so
    // by the time a new frame arrives the robot has already turned further.
    // Measure how fast the error is moving (deg/sec) and aim at where the
    // target WILL be after this lookahead, so the turn eases before it
    // overshoots instead of after.
    private static final double AIM_LOOKAHEAD_S    = 0.16;
    private static final double AIM_RATE_ALPHA     = 0.35;   // 0 = frozen, 1 = raw
    private static final double AIM_MAX_RATE_DPS   = 300.0;

    // Fine zone: below this error do NOT turn continuously. The camera only
    // updates ~40x/sec, so even a slow continuous turn sweeps past the target
    // between frames. Turn for a short pulse instead, then brake and let the
    // next frame re-measure. This is what allows a small deadband without
    // overshoot.
    private static final double FINE_ZONE_DEG      = 6.0;
    private static final double FINE_PULSE_POWER   = 0.12;
    private static final double FINE_MS_PER_DEG    = 12.0;
    private static final double FINE_PULSE_MAX_MS  = 70.0;
    private static final long   FINE_COOLDOWN_MS   = 100;

    // ---------------------------------------------------------------------
    // Turning toward the tag -- the drivetrain half
    // ---------------------------------------------------------------------
    // YAW_SIGN is the ONE place the turn sign is inverted. The targeting code
    // speaks "positive = clockwise", and the motors on this drivetrain are
    // wired so that clockwise needs a negative contribution on the left side.
    // Everything above this line therefore stays sign-free.
    private static final double YAW_SIGN       = -1.0;

    // Hard cap on the pivot, independent of the aim maths. A mis-solved pose or
    // a tag flashing past the edge of the view can otherwise ask for full
    // power; 0.35 is fast enough to be useful and slow enough to stay safe in
    // a small test space.
    private static final double TURN_MAX_POWER = 0.35;

    // ---------------------------------------------------------------------
    // State
    // ---------------------------------------------------------------------
    private Limelight3A limelight;
    private boolean     limelightReady;
    private String      limelightError  = "";
    private String      limelightSource = "";
    private int         pipeline        = START_PIPELINE;

    // The live ID gate. Starts at the constant above and is changed by the
    // bumpers. -1 means "any tag".
    private int         idGate          = TARGET_ID_GATE;

    private LLStatus cachedStatus;
    private long     cachedStatusMs;
    private boolean  cachedStatusValid;
    private long     lastLogMs;

    // Last non-empty frame, reused briefly so the display does not flicker.
    private List<LLTarget> heldTargets;
    private long           heldMs;

    // Aim predictor state.
    private boolean aimRateValid;
    private double  aimRateDps;
    private double  lastAimError;
    private long    lastAimMs;
    private long    pulseUntilMs;
    private double  pulseSign;
    // Values from the most recent aimTurn() call, kept for telemetry.
    private double  aimError;
    private double  aimPredicted;

    // Only used by STRATEGY C. Harmless when that block is deleted.
    private final Random random = new Random();

    // Drivetrain. driveReady stays false when the four motors are not all in
    // the robot configuration; the camera half then runs on its own.
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;
    private boolean driveReady;
    private String  driveError = "";

    // Turning is armed by Y, never at START, so the robot cannot move before
    // the driver has seen the camera numbers.
    private boolean turnEnabled;

    private boolean prevDpadUp, prevDpadDown, prevA, prevB, prevX, prevY;
    // Bumper edge detection, used to step the ID gate.
    private boolean prevLeftBumper, prevRightBumper;

    // ---------------------------------------------------------------------
    // OpMode
    // ---------------------------------------------------------------------

    @Override
    public void runOpMode() {
        initLimelight();
        initDrive();

        // Live values while sitting at INIT, so the pipeline can be checked
        // before START.
        while (!isStarted() && !isStopRequested()) {
            LLResult raw = poll();
            show(raw, null, null, 0.0, 0.0);
            telemetry.update();
            sleep(50);
        }

        try {
            while (opModeIsActive()) {
                long now = System.currentTimeMillis();
                LLResult raw = poll();
                // The gate is applied to the RAW frame, so the bumper cycle
                // below always sees every tag that is actually in view.
                handleButtons(raw);

                List<LLTarget> targets = buildTargets(raw, idGate);
                if (!targets.isEmpty()) {
                    heldTargets = targets;
                    heldMs = now;
                } else if (heldTargets != null && now - heldMs <= HOLD_LAST_MS) {
                    targets = heldTargets;
                } else {
                    heldTargets = null;
                }

                // The strategy decides the order; the first entry is the pick.
                orderTargets(targets);
                LLTarget chosen = targets.isEmpty() ? null : targets.get(0);

                double turn = 0.0;
                if (chosen != null) {
                    turn = aimTurn(chosen, now);
                } else {
                    // No target: forget the learned rate so the next acquire
                    // starts from rest instead of inheriting a stale lead.
                    aimRateValid = false;
                    pulseUntilMs = 0;
                }

                // Only now does anything reach a motor. Without a target, or
                // without Y, the wheels are explicitly zeroed every frame so
                // a dropped tag can never leave the robot spinning.
                double applied = applyDrive(turn, chosen != null);

                show(raw, targets, chosen, turn, applied);
                log(raw, targets, chosen, turn, applied, now);
                telemetry.update();
            }
        } finally {
            stopDrive();
            stopLimelight();
        }
    }

    // ---------------------------------------------------------------------
    // Limelight connection
    // ---------------------------------------------------------------------

    private void initLimelight() {
        // 1) Normal path: the Limelight is in the robot configuration.
        for (String name : LIMELIGHT_NAMES) {
            try {
                Limelight3A found = hardwareMap.tryGet(Limelight3A.class, name);
                if (found != null) {
                    limelight = found;
                    limelightSource = "robot config \"" + name + "\"";
                    break;
                }
            } catch (RuntimeException ignored) {
                // Wrong type or not present; try the next name.
            }
        }

        // 2) Any Limelight3A in the configuration, whatever it is called.
        if (limelight == null) {
            try {
                List<Limelight3A> all = hardwareMap.getAll(Limelight3A.class);
                if (all != null && !all.isEmpty()) {
                    limelight = all.get(0);
                    limelightSource = "robot config (auto-found)";
                }
            } catch (RuntimeException ignored) {
                // Fall through to the direct connection.
            }
        }

        // 3) Direct connection for configurations that do not list it yet.
        if (limelight == null) {
            limelight = connectDirectLimelight();
        }

        if (limelight == null) {
            limelightReady = false;
            if (limelightError.isEmpty()) {
                limelightError = "No Limelight3A found (config or direct)";
            }
            return;
        }

        try {
            limelight.setPollRateHz(POLL_RATE_HZ);
            limelight.pipelineSwitch(pipeline);
            limelight.start();      // getLatestResult() is empty until this runs
            limelightReady = true;
        } catch (RuntimeException e) {
            limelightError = "Start failed: " + describe(e);
            limelightReady = false;
        }
    }

    /**
     * Creates a Limelight3A without using the robot configuration, over the
     * hub's Ethernet-over-USB interface. Reflection keeps this file compiling
     * on SDK versions that do not expose the internal serial-number class.
     */
    private Limelight3A connectDirectLimelight() {
        String localIp = null;
        String ifaceName = null;

        try {
            Enumeration<NetworkInterface> interfaces = NetworkInterface.getNetworkInterfaces();
            while (interfaces.hasMoreElements()) {
                NetworkInterface ni = interfaces.nextElement();
                if (!ni.isUp() || ni.isLoopback()) {
                    continue;
                }
                for (InterfaceAddress ia : ni.getInterfaceAddresses()) {
                    InetAddress address = ia.getAddress();
                    if (address instanceof Inet4Address) {
                        String ip = address.getHostAddress();
                        if (ip.startsWith(LIMELIGHT_SUBNET_PREFIX) && !ip.equals(LIMELIGHT_IP)) {
                            localIp = ip;
                            ifaceName = ni.getName();
                        }
                    }
                }
            }
        } catch (Exception e) {
            limelightError = "Network scan failed: " + e.getMessage();
            return null;
        }

        if (localIp == null) {
            limelightError = "Limelight not on " + LIMELIGHT_SUBNET_PREFIX
                    + "x.x; check the USB cable";
            return null;
        }

        try {
            Class<?> serialClass = Class.forName(
                    "org.firstinspires.ftc.robotcore.internal.usb.EthernetOverUsbSerialNumber");
            Object serial = serialClass
                    .getMethod("fromIpAddress", String.class, String.class)
                    .invoke(null, localIp, ifaceName);

            Constructor<Limelight3A> ctor = Limelight3A.class.getConstructor(
                    SerialNumber.class, String.class, InetAddress.class);
            Limelight3A direct = ctor.newInstance(
                    serial, LIMELIGHT_OBJECT_NAME, InetAddress.getByName(LIMELIGHT_IP));

            limelightSource = "direct " + ifaceName + " " + localIp + " -> " + LIMELIGHT_IP;
            return direct;
        } catch (Throwable t) {
            limelightError = "Direct connect failed: " + describe(t);
            return null;
        }
    }

    private void stopLimelight() {
        if (limelight != null) {
            try {
                limelight.stop();
            } catch (RuntimeException ignored) {
                // Nothing useful to do while shutting down.
            }
        }
    }

    private static String describe(Throwable t) {
        String message = t.getMessage();
        return t.getClass().getSimpleName() + (message == null ? "" : ": " + message);
    }

    // ---------------------------------------------------------------------
    // Drivetrain -- turning toward the tag
    // ---------------------------------------------------------------------

    /**
     * Finds the four drive motors by candidate name and sets them up.
     *
     * <p>A missing motor is not fatal: the camera half of this OpMode is
     * useful on its own, so the failure is recorded and the drive is simply
     * never touched.
     */
    private void initDrive() {
        leftFront  = findMotor(LEFT_FRONT_NAMES);
        leftRear   = findMotor(LEFT_REAR_NAMES);
        rightFront = findMotor(RIGHT_FRONT_NAMES);
        rightRear  = findMotor(RIGHT_REAR_NAMES);

        if (leftFront == null || leftRear == null
                || rightFront == null || rightRear == null) {
            driveReady = false;
            driveError = "missing " + missingMotors();
            return;
        }
        // The left side is mounted mirrored, so it runs REVERSE. This matches
        // the stock mecanum configuration and "Limelight: Test"; if a robot
        // pivots the wrong way, these four lines are the place to look.
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        // BRAKE, not FLOAT: a pivot that coasts past the deadband is exactly
        // the overshoot the fine-zone pulses are there to avoid.
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stopDrive();
        driveReady = true;
    }

    /** First name in the list that exists in the configuration, or null. */
    private DcMotor findMotor(String[] names) {
        for (String name : names) {
            try {
                DcMotor motor = hardwareMap.tryGet(DcMotor.class, name);
                if (motor != null) {
                    return motor;
                }
            } catch (RuntimeException ignored) {
                // Wrong type or not present; try the next name.
            }
        }
        return null;
    }

    /** "leftFront, rightRear" -- the ones that could not be found. */
    private String missingMotors() {
        StringBuilder sb = new StringBuilder();
        appendMissing(sb, "leftFront", leftFront);
        appendMissing(sb, "leftRear", leftRear);
        appendMissing(sb, "rightFront", rightFront);
        appendMissing(sb, "rightRear", rightRear);
        return sb.toString();
    }

    private static void appendMissing(StringBuilder sb, String name, DcMotor motor) {
        if (motor != null) {
            return;
        }
        if (sb.length() > 0) {
            sb.append(", ");
        }
        sb.append(name);
    }

    /**
     * Every DcMotor name in the robot configuration, sorted, or "none".
     *
     * <p>Printed only when the drive could not be built, because that is the
     * one case where "which motors does the hub actually see?" is the whole
     * question. It reports the names in the configuration, not the names this
     * file looks for, so a spelling difference is immediately obvious.
     */
    private String configuredMotorNames() {
        try {
            SortedSet<String> names = hardwareMap.getAllNames(DcMotor.class);
            if (names == null || names.isEmpty()) {
                return "none";
            }
            StringBuilder sb = new StringBuilder();
            for (String name : names) {
                if (sb.length() > 0) {
                    sb.append(", ");
                }
                sb.append(name);
            }
            return sb.toString();
        } catch (RuntimeException e) {
            return "unreadable (" + describe(e) + ")";
        }
    }

    /**
     * Sends the aim turn to the wheels, or stops them.
     *
     * <p>Three things must all be true before a motor moves: the four motors
     * exist, the driver has armed turning with Y, and this frame produced a
     * target. Anything else is a stop, not a hold -- holding the last power
     * would keep the robot turning after the tag left the view.
     *
     * @param turn       the aim command, positive = clockwise
     * @param haveTarget whether this frame produced a target to aim at
     * @return the power actually sent (0 whenever the wheels were stopped)
     */
    private double applyDrive(double turn, boolean haveTarget) {
        if (!driveReady) {
            return 0.0;
        }
        if (!turnEnabled || !haveTarget) {
            stopDrive();
            return 0.0;
        }
        double applied = clamp(turn, -TURN_MAX_POWER, TURN_MAX_POWER);
        drive(0.0, 0.0, applied);
        return applied;
    }

    /**
     * Mecanum mixing. Positive yaw turns clockwise (to the robot's right).
     *
     * <p>Axial and lateral are accepted so this method is drop-in usable from
     * a driving OpMode, but the targeting loop only ever passes zeros.
     */
    private void drive(double axial, double lateral, double yaw) {
        double y = yaw * YAW_SIGN;

        double fl = axial + lateral + y;
        double fr = axial - lateral - y;
        double bl = axial - lateral + y;
        double br = axial + lateral - y;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        leftFront.setPower(fl);
        rightFront.setPower(fr);
        leftRear.setPower(bl);
        rightRear.setPower(br);
    }

    private void stopDrive() {
        if (leftFront != null) {
            leftFront.setPower(0.0);
        }
        if (leftRear != null) {
            leftRear.setPower(0.0);
        }
        if (rightFront != null) {
            rightFront.setPower(0.0);
        }
        if (rightRear != null) {
            rightRear.setPower(0.0);
        }
    }

    // ---------------------------------------------------------------------
    // Polling
    // ---------------------------------------------------------------------

    private LLResult poll() {
        if (!limelightReady) {
            return null;
        }
        try {
            return limelight.getLatestResult();
        } catch (RuntimeException e) {
            limelightError = "Poll failed: " + describe(e);
            return null;
        }
    }

    /** getStatus() is an HTTP call, so keep a short-lived copy of it. */
    private LLStatus cachedStatus() {
        if (limelight == null || !limelightReady) {
            return null;
        }
        long now = System.currentTimeMillis();
        if (!cachedStatusValid || now - cachedStatusMs > STATUS_REFRESH_MS) {
            cachedStatus = limelight.getStatus();
            cachedStatusMs = now;
            cachedStatusValid = cachedStatus != null;
        }
        return cachedStatus;
    }

    // ---------------------------------------------------------------------
    // Turning the raw result into LLTarget objects
    // ---------------------------------------------------------------------

    /**
     * One entry per AprilTag in this frame that passes the ID gate, or an
     * empty list.
     *
     * <p>An "invalid" result is not an error: it just means the pipeline found
     * nothing this time, which is normal when no tag is in view.
     *
     * <p>The gate is a parameter rather than a field read so this method stays
     * pure and reusable: pass -1 for "any tag", or an ID to keep only that one.
     * A tag that fails the gate is dropped here, before ordering, so a
     * strategy never needs to know the gate exists.
     *
     * @param raw    the SDK result for this frame, possibly null
     * @param idGate -1 for any tag, otherwise the only allowed tag ID
     */
    private static List<LLTarget> buildTargets(LLResult raw, int idGate) {
        List<LLTarget> out = new ArrayList<>();
        if (raw == null || !raw.isValid()
                || raw.getStaleness() > MAX_RESULT_AGE_MS) {
            return out;                 // a stale frame counts as no target
        }
        List<LLResultTypes.FiducialResult> tags = raw.getFiducialResults();
        if (tags == null) {
            return out;
        }

        for (LLResultTypes.FiducialResult tag : tags) {
            LLTarget t = new LLTarget();
            t.id         = tag.getFiducialId();
            if (idGate >= 0 && t.id != idGate) {
                continue;               // gated out: not a candidate
            }
            t.label      = nullToDash(tag.getFamily()) + " #" + t.id;
            t.tx         = tag.getTargetXDegrees();
            t.txnc       = tag.getTargetXDegreesNoCrosshair();
            t.ty         = tag.getTargetYDegrees();
            t.tync       = tag.getTargetYDegreesNoCrosshair();
            t.area       = tag.getTargetArea();
            t.rangeM     = tagRangeM(tag);
            t.tagYawDeg  = tagYawDeg(tag);
            t.stalenessMs = raw.getStaleness();
            out.add(t);
        }
        return out;
    }

    /**
     * Range to the tag from the Limelight's own pose solve, in metres.
     *
     * <p>AprilTag pipelines solve the full 6-DoF pose, so this is far better
     * than any area-based guess. Camera space is +x right, +y down, +z
     * forward, so the straight-line distance is hypot(x, z).
     *
     * @return the range, or a negative value if there is no pose solve.
     */
    private static double tagRangeM(LLResultTypes.FiducialResult tag) {
        Pose3D pose = tag.getTargetPoseCameraSpace();
        if (pose == null || pose.getPosition() == null) {
            return -1.0;
        }
        Position p = pose.getPosition().toUnit(DistanceUnit.METER);
        return Math.hypot(p.x, p.z);
    }

    /** The tag's yaw in camera space, degrees, or NaN without a pose. */
    private static double tagYawDeg(LLResultTypes.FiducialResult tag) {
        Pose3D pose = tag.getTargetPoseCameraSpace();
        if (pose == null || pose.getOrientation() == null) {
            return Double.NaN;
        }
        return pose.getOrientation().getYaw(AngleUnit.DEGREES);
    }

    // ---------------------------------------------------------------------
    // ID gate helpers
    // ---------------------------------------------------------------------

    /**
     * Every distinct tag ID in this frame, sorted ascending, with the gate NOT
     * applied.
     *
     * <p>This is what the bumper cycle walks. It deliberately ignores the
     * current gate: if the gate hid a tag, that tag must still be reachable by
     * pressing the bumper, otherwise a bad gate would be a dead end.
     *
     * <p>A stale frame returns an empty list, so the cycle cannot offer IDs
     * that are no longer in view.
     */
    private static List<Integer> visibleIds(LLResult raw) {
        List<Integer> ids = new ArrayList<>();
        if (raw == null || !raw.isValid()
                || raw.getStaleness() > MAX_RESULT_AGE_MS) {
            return ids;
        }
        List<LLResultTypes.FiducialResult> tags = raw.getFiducialResults();
        if (tags == null) {
            return ids;
        }
        for (LLResultTypes.FiducialResult tag : tags) {
            int id = tag.getFiducialId();
            if (!ids.contains(id)) {
                ids.add(id);
            }
        }
        Collections.sort(ids);
        return ids;
    }

    /**
     * Step the gate to the next entry in the cycle
     * {@code [-1, id0, id1, ...]}, where the ids are the ones currently in
     * view.
     *
     * <p>-1 ("any") is always the first stop, so pressing the bumper enough
     * times always returns to "no gate" without needing the tags to be there.
     * If the current gate is an ID that has since left the view, the walk
     * restarts from the "any" stop, so a press always lands on something real
     * instead of getting stuck.
     *
     * @param direction +1 for the next stop, -1 for the previous one
     */
    private void cycleGate(LLResult raw, int direction) {
        List<Integer> stops = new ArrayList<>();
        stops.add(-1);                          // "any" is always reachable
        stops.addAll(visibleIds(raw));          // then each ID actually in view

        int index = stops.indexOf(idGate);
        if (index < 0) {
            // The gate is an ID that is not in view. Start from "any" so the
            // next press lands somewhere sensible.
            index = 0;
        }

        int count = stops.size();
        int next = ((index + direction) % count + count) % count;

        idGate = stops.get(next);
        heldTargets = null;                     // old pick may be gated out
        System.out.println("[Target] id gate -> "
                + (idGate < 0 ? "any" : "#" + idGate)
                + "  (visible " + idList(raw) + ")");
    }

    /** "any", "#3", "#3 #5" -- for telemetry and the log. */
    private static String idList(LLResult raw) {
        List<Integer> ids = visibleIds(raw);
        if (ids.isEmpty()) {
            return "none";
        }
        StringBuilder sb = new StringBuilder();
        for (int id : ids) {
            if (sb.length() > 0) {
                sb.append(' ');
            }
            sb.append('#').append(id);
        }
        return sb.toString();
    }

    // =====================================================================
    // TARGET ORDERING -- the three interchangeable strategies
    // =====================================================================
    //
    // Exactly one of the three blocks below is uncommented. Each block
    // contains BOTH the ORDER_NAME string and the orderTargets() method, so
    // deleting a block removes everything that belonged to that strategy.
    //
    // orderTargets() sorts the list IN PLACE. The caller then takes get(0).
    // An empty or single-entry list is fine for all three.
    //
    // Note on stability: none of these strategies remembers what it picked
    // last frame. If two tags are nearly identical in size, BY SIZE can
    // alternate between them from frame to frame. That is deliberate here --
    // this is the "basic" version. To make it sticky, keep the last chosen
    // id and only switch when a different tag beats it by some margin.
    // =====================================================================

    // ---------------------------------------------------------------------
    // STRATEGY A -- BY SIZE
    // ---------------------------------------------------------------------
    // Biggest apparent area first. Area grows as the tag gets closer, so this
    // is effectively "nearest tag first". Use it when you want the robot to
    // go for the tag it can see best.
    //
    // ta comes straight from the Limelight and is NOT normalised here, so
    // only the ORDER matters, never the absolute number.
    private static final String ORDER_NAME = "A - by size (largest area first)";

    private static void orderTargets(List<LLTarget> targets) {
        Collections.sort(targets, new Comparator<LLTarget>() {
            @Override
            public int compare(LLTarget a, LLTarget b) {
                return Double.compare(b.area, a.area);   // biggest first
            }
        });
    }
    // ---------------------------------------------------------------------
    // END STRATEGY A
    // ---------------------------------------------------------------------

    // ---------------------------------------------------------------------
    // STRATEGY B -- BY POSITION
    // ---------------------------------------------------------------------
    // Smallest |txnc| first, i.e. the tag nearest the middle of the camera.
    // That is the tag needing the least rotation, so it is the cheapest to
    // aim at. Ties are broken by area so the order is deterministic.
    //
    // To order left-to-right across the view instead, change the compare()
    // body below to:  return Double.compare(a.txnc, b.txnc);
    //
    // Everything below the dashed line is commented-out code. Select it and
    // uncomment it to use this strategy.
    // ---------------------------------------------------------------------
    // private static final String ORDER_NAME = "B - by position (most centred)";
    //
    // private static void orderTargets(List<LLTarget> targets) {
    //     Collections.sort(targets, new Comparator<LLTarget>() {
    //         @Override
    //         public int compare(LLTarget a, LLTarget b) {
    //             int byAngle = Double.compare(Math.abs(a.txnc), Math.abs(b.txnc));
    //             if (byAngle != 0) {
    //                 return byAngle;
    //             }
    //             return Double.compare(b.area, a.area);
    //         }
    //     });
    // }
    // ---------------------------------------------------------------------
    // END STRATEGY B
    // ---------------------------------------------------------------------

    // ---------------------------------------------------------------------
    // STRATEGY C -- RANDOM
    // ---------------------------------------------------------------------
    // Shuffles the list every frame. This is a TEST strategy, not a control
    // strategy: the pick will jump between tags while several are visible.
    // It is here to prove the pipeline really sees every tag and that the
    // rest of the targeting code does not secretly depend on the order.
    //
    // If you want a random pick that HOLDS, remember the chosen id and keep
    // returning it until it disappears, instead of shuffling each frame.
    //
    // Everything below the dashed line is commented-out code. Select it and
    // uncomment it to use this strategy. This one is not static, because it uses the `random` field
    //
    // ---------------------------------------------------------------------
    // private static final String ORDER_NAME = "C - random (reshuffled every frame)";
    //
    // private void orderTargets(List<LLTarget> targets) {
    //     Collections.shuffle(targets, random);
    // }
    // ---------------------------------------------------------------------
    // END STRATEGY C

    // =====================================================================
    // AIM
    // =====================================================================

    /**
     * The turn command that would centre this target, in -1..1.
     *
     * <p>Positive means "rotate clockwise / to the right", matching a target
     * that sits to the right of the camera centre. Nothing is applied to a
     * motor here; the value is returned so it can be printed or handed to a
     * drivetrain.
     *
     * <p>The maths is the overshoot predictor from "Limelight: Test":
     * a smoothed error rate extrapolates where the target will be after the
     * camera latency, and inside the fine zone the output becomes short
     * pulses so a small deadband does not cost a sweep past the target.
     */
    private double aimTurn(LLTarget target, long now) {
        double error = target.txnc;
        aimError = error;

        // --- measure how fast the error is moving -----------------------
        if (!aimRateValid) {
            // First frame of a new aim: start from rest.
            aimRateDps = 0.0;
            lastAimError = error;
            lastAimMs = now;
            aimRateValid = true;
        } else {
            double delta = error - lastAimError;
            if (Math.abs(delta) > 0.02) {
                double dt = (now - lastAimMs) / 1000.0;
                if (dt > 0.02) {
                    double raw = delta / dt;
                    aimRateDps = AIM_RATE_ALPHA * raw
                            + (1.0 - AIM_RATE_ALPHA) * aimRateDps;
                    aimRateDps = clamp(aimRateDps, -AIM_MAX_RATE_DPS, AIM_MAX_RATE_DPS);
                }
                lastAimError = error;
                lastAimMs = now;
            } else {
                // Nothing moved this poll; let a stale lead decay so it
                // cannot keep the robot turning.
                aimRateDps *= 0.90;
            }
        }

        // --- extrapolate ------------------------------------------------
        double lookahead = AIM_LOOKAHEAD_S + target.stalenessMs / 1000.0;
        double predicted = error + aimRateDps * lookahead;

        // Never let the lead command a turn back the other way while the
        // target is still off-centre: brake instead.
        if ((error > 0.0 && predicted < 0.0)
                || (error < 0.0 && predicted > 0.0)) {
            predicted = 0.0;
        }
        aimPredicted = predicted;

        // --- decide -----------------------------------------------------
        if (Math.abs(predicted) <= TX_DEADBAND_DEG) {
            pulseUntilMs = 0;
            return 0.0;                                  // on target
        }

        if (Math.abs(predicted) <= FINE_ZONE_DEG) {
            // Fine zone: step in short pulses. Start a new pulse once the
            // previous pulse and its cooldown are done, so each pulse gets a
            // fresh camera frame to be judged by.
            if (now >= pulseUntilMs + FINE_COOLDOWN_MS) {
                double pulseMs = Math.min(
                        Math.abs(predicted) * FINE_MS_PER_DEG, FINE_PULSE_MAX_MS);
                pulseUntilMs = now + (long) pulseMs;
                pulseSign = Math.signum(predicted);
            }
            return (now < pulseUntilMs) ? pulseSign * FINE_PULSE_POWER : 0.0;
        }

        // Coarse zone: continuous turn with the predictor.
        pulseUntilMs = 0;
        double turn = predicted * AIM_KP;
        if (Math.abs(turn) < AIM_MIN_POWER) {
            turn = Math.copySign(AIM_MIN_POWER, turn);
        }
        double limit = Math.abs(predicted) < AIM_SLOW_ZONE_DEG
                ? AIM_SLOW_MAX_POWER
                : AIM_MAX_POWER;
        return clamp(turn, -limit, limit);
    }

    // ---------------------------------------------------------------------
    // Controls
    // ---------------------------------------------------------------------

    private void handleButtons(LLResult raw) {
        // The drive toggle is handled first, on purpose: turning must be
        // armable/disarmable even when the camera is down, otherwise a broken
        // Limelight would leave the robot with no way to stop it.
        if (gamepad1.y && !prevY) {
            turnEnabled = !turnEnabled;
            if (!turnEnabled) {
                stopDrive();
            }
            System.out.println("[Target] turn " + (turnEnabled ? "ARMED" : "disarmed"));
        }
        prevY = gamepad1.y;

        if (!limelightReady) {
            return;
        }

        if (gamepad1.dpad_up && !prevDpadUp) {
            changePipeline(1);
        }
        if (gamepad1.dpad_down && !prevDpadDown) {
            changePipeline(-1);
        }
        // Bumpers step the ID gate: RB forward, LB back. The raw frame is
        // passed in so the cycle can offer the IDs that are actually in view.
        if (gamepad1.right_bumper && !prevRightBumper) {
            cycleGate(raw, 1);
        }
        if (gamepad1.left_bumper && !prevLeftBumper) {
            cycleGate(raw, -1);
        }
        if (gamepad1.b && !prevB) {
            limelight.reloadPipeline();
            System.out.println("[Target] reload pipeline " + pipeline);
        }
        if (gamepad1.x && !prevX) {
            heldTargets = null;
            System.out.println("[Target] held target dropped");
        }
        if (gamepad1.a && !prevA) {
            limelight.stop();
            limelight.start();
            System.out.println("[Target] stream restarted");
        }

        prevDpadUp      = gamepad1.dpad_up;
        prevDpadDown    = gamepad1.dpad_down;
        prevLeftBumper  = gamepad1.left_bumper;
        prevRightBumper = gamepad1.right_bumper;
        prevA = gamepad1.a;
        prevB = gamepad1.b;
        prevX = gamepad1.x;
    }

    private void changePipeline(int delta) {
        pipeline = (pipeline + delta + PIPELINE_COUNT) % PIPELINE_COUNT;
        limelight.pipelineSwitch(pipeline);
        heldTargets = null;
        System.out.println("[Target] pipeline " + pipeline);
    }

    // ---------------------------------------------------------------------
    // Telemetry + log
    // ---------------------------------------------------------------------

    private void show(LLResult raw, List<LLTarget> ordered, LLTarget chosen,
                      double turn, double applied) {
        telemetry.addData("Limelight", limelightReady
                ? "OK  " + limelightSource
                : "NOT READY " + limelightError);

        // The drive line is deliberately near the top: it is the state that
        // decides whether the robot can move.
        telemetry.addData("Turn", driveReady
                ? String.format("%s (Y)  sent %+.3f  cap %.2f",
                turnEnabled ? "ARMED" : "off", applied, TURN_MAX_POWER)
                : "no drive: " + driveError);
        if (!driveReady) {
            // The names the hub really has, so a typo in the config shows up
            // next to the names this file wanted.
            telemetry.addData("Config motors", configuredMotorNames());
        }

        LLStatus status = cachedStatus();
        if (status != null) {
            telemetry.addData("Camera", String.format(
                    "pipe %d (%s)  fps %.0f  %.0f C  cpu %.0f%%",
                    status.getPipelineIndex(), nullToDash(status.getPipelineType()),
                    status.getFps(), status.getTemp(), status.getCpu()));
        } else {
            telemetry.addData("Camera", "no status yet");
        }

        if (raw == null) {
            telemetry.addData("Result", "no result (is the stream started?)");
        } else {
            telemetry.addData("Result", String.format(
                    "%s  pipe %d (%s)  %d ms old  cap %.1f ms  tags %d",
                    raw.isValid() ? "live" : "empty",
                    raw.getPipelineIndex(),
                    nullToDash(raw.getPipelineType()),
                    raw.getStaleness(),
                    raw.getCaptureLatency(),
                    countTags(raw)));
        }

        telemetry.addData("Order", ORDER_NAME);

        // The gate, and every ID that is actually in view. When those two
        // disagree, "Target: none" is the gate working, not a camera fault.
        telemetry.addData("Gate", String.format(
                "%s  (visible %s)  LB/RB to change",
                idGate < 0 ? "any" : "#" + idGate,
                idList(raw)));

        if (chosen == null) {
            telemetry.addData("Target", "none");
            telemetry.addData("Aim", "-");
            telemetry.addData("List", idGate < 0
                    ? "no tags in view"
                    : "gate #" + idGate + " matched nothing in this frame");
            return;
        }

        telemetry.addData("Target", String.format(
                "%s  tx %+.1f  ty %+.1f  ta %.4f  range %s",
                chosen.label, chosen.tx, chosen.ty, chosen.area,
                fmtRange(chosen.rangeM)));

        telemetry.addData("Aim", String.format(
                "err %+.1f  rate %+.0f dps  pred %+.1f  turn %+.3f",
                aimError, aimRateDps, aimPredicted, turn));

        // The whole ordered list, so it is obvious what the strategy did.
        StringBuilder sb = new StringBuilder();
        int shown = 0;
        for (LLTarget t : ordered) {
            if (shown >= MAX_LISTED) {
                sb.append(" ...");
                break;
            }
            if (shown > 0) {
                sb.append("  ");
            }
            sb.append(String.format("#%d %.3f", t.id, t.area));
            shown++;
        }
        telemetry.addData("List", sb.toString());
    }

    private void log(LLResult raw, List<LLTarget> ordered, LLTarget chosen,
                     double turn, double applied, long now) {
        if (now - lastLogMs < LOG_INTERVAL_MS || raw == null) {
            return;
        }
        lastLogMs = now;

        System.out.println(String.format(
                "[Target] %s | pipe=%d tags=%d gate=%s chosen=%s turn=%.3f %s%s",
                ORDER_NAME,
                raw.getPipelineIndex(),
                countTags(raw),
                idGate < 0 ? "any" : "#" + idGate,
                chosen == null ? "none" : chosen.label,
                turn,
                turnEnabled ? "armed" : "off",
                driveReady ? "" : " (no drive)"));

        for (LLTarget t : ordered) {
            System.out.println(String.format(
                    "[Target]   %s tx=%.1f ty=%.1f area=%.4f range=%.2f yaw=%.0f",
                    t.label, t.tx, t.ty, t.area, t.rangeM, t.tagYawDeg));
        }
    }

    private static int countTags(LLResult raw) {
        if (raw == null || !raw.isValid()) {
            return 0;
        }
        List<LLResultTypes.FiducialResult> tags = raw.getFiducialResults();
        return tags == null ? 0 : tags.size();
    }

    // ---------------------------------------------------------------------
    // Small helpers
    // ---------------------------------------------------------------------

    private static String nullToDash(String value) {
        return value == null || value.isEmpty() ? "-" : value;
    }

    private static String fmtRange(double metres) {
        return (metres > 0.0 && metres < 1000.0)
                ? String.format("%.2f m", metres)
                : "--";
    }

    private static double clamp(double value, double lo, double hi) {
        return value < lo ? lo : (value > hi ? hi : value);
    }

    // ---------------------------------------------------------------------
    // Data
    // ---------------------------------------------------------------------

    /**
     * One AprilTag, as seen in a single frame.
     *
     * <p>Deliberately a plain data holder: no behaviour, no history. Anything
     * that needs memory (the aim rate) lives in the OpMode, so this class
     * stays easy to copy into another file.
     *
     * <p>Named LLTarget rather than Target because OnBotJava's auto-import
     * likes to add "import java.lang.annotation.Target" when it sees that
     * bare name, which then breaks the file.
     */
    private static class LLTarget {
        int    id;             // fiducial id
        String label;          // e.g. "36h11 #3"
        double tx;             // angle to the crosshair, degrees
        double txnc;           // angle to the camera centre, degrees  <-- aim
        double ty;             // vertical angle to the crosshair
        double tync;           // vertical angle to the camera centre
        double area;           // raw ta from the Limelight, not normalised
        double rangeM;         // metres, negative when there is no pose solve
        double tagYawDeg;      // tag yaw in camera space, NaN without a pose
        double stalenessMs;    // age of the frame this came from
    }
}

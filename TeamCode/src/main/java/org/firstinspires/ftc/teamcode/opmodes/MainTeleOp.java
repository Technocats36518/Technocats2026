package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterIndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;


@TeleOp(name = "Mecanum Ivy Dual TeleOp V2", group = "TeleOp")
public class MainTeleOp extends OpMode {

    private Follower follower;
    private IntakeSubsystem intake;
    private ShooterIndexerSubsystem shooter;
    private VisionSubsystem vision;

    private enum Alliance { RED, BLUE }
    private Alliance alliance = Alliance.RED;
    private boolean bLastInit = false;
    private boolean xLastInit = false;

    // Hive pivot midpoints in field inches (per alliance)
    private static final double RED_HIVE_X = 58.0, RED_HIVE_Y = 71.5;
    private static final double BLUE_HIVE_X = 84.0, BLUE_HIVE_Y = 71.5;

    // Calibration table: distance (inches) -> flywheel power
    private static final double[] DIST_IN = {24, 42, 50, 55};
    private static final double[] POWER   = {0.44, 0.54, 0.64, 0.74};

    // Hold-to-aim tuning
    private static final double AIM_KP = 0.020;          // turn power per degree of tx
    private static final double AIM_LOCK_DEG = 1.5;      // must be inside this to lock
    private static final double AIM_RELEASE_DEG = 4.0;   // stays locked until outside this
    private static final double AIM_FILTER = 0.3;        // 0..1, lower = smoother
    private static final long AIM_LOST_HOLD_MS = 150;    // keep last angle this long after tag drops
    private static final double AIM_MAX_TURN = 0.4;

    private double txFiltered = 0.0;
    private boolean txValid = false;
    private boolean aimLocked = false;
    private long lastTagSeenMs = 0;

    private double headingOffset = 0.0;   // radians
    private boolean startLast = false;

    // Runtime state variables to handle button edge-triggers safely
    private boolean rightBumper2Last = false;
    private boolean cross2Last = false;
    private boolean isShooterActive = false;

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);

        // Use the pose saved by the autonomous, or the origin if none was saved
        Pose savedPose = OpModeStorage.autonomousEndPose != null
                ? OpModeStorage.autonomousEndPose
                : new Pose(0.0, 0.0, 0.0);
        follower.setPose(savedPose);

        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterIndexerSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);

        alliance = OpModeStorage.isRedAlliance ? Alliance.RED : Alliance.BLUE;
        headingOffset = follower.pose().heading();
    }

    @Override
    public void init_loop() {
        if (gamepad1.b && !bLastInit) alliance = Alliance.RED;
        if (gamepad1.x && !xLastInit) alliance = Alliance.BLUE;
        bLastInit = gamepad1.b;
        xLastInit = gamepad1.x;

        telemetry.addData("Alliance", alliance == Alliance.RED ? "RED (B)" : "BLUE (X)");
        telemetry.addData("Press B for red, X for blue", "");
        telemetry.update();
    }

    @Override
    public void start() {
        vision.startCamera();
    }

    @Override
    public void loop() {

        // Driver presses START while the robot faces away from them
        if (gamepad1.start && !startLast) {
            headingOffset = follower.pose().heading();
        }
        startLast = gamepad1.start;

        // ==========================================
        // GAMEPAD 1: DRIVETRAIN WITH HOLD-TO-AIM
        // ==========================================
        // Hold left bumper to rotate the robot onto the hive tags.
        // While held, the right stick rotation is ignored.
        int[] tags = hiveTags();
        double turn = gamepad1.right_stick_x;
        boolean aimed = false;

        if (gamepad1.left_bumper) {
            long now = System.currentTimeMillis();
            double tx = vision.averageTx(tags);

            if (!Double.isNaN(tx)) {
                // Smooth the raw angle so single noisy frames don't move the robot
                txFiltered = txValid ? txFiltered + AIM_FILTER * (tx - txFiltered) : tx;
                txValid = true;
                lastTagSeenMs = now;
            } else if (now - lastTagSeenMs > AIM_LOST_HOLD_MS) {
                // Tag gone for too long, so stop trusting the old angle
                txValid = false;
            }

            if (txValid) {
                double a = Math.abs(txFiltered);
                // Hysteresis: lock tightly, release only when clearly off target
                aimLocked = aimLocked ? (a < AIM_RELEASE_DEG) : (a < AIM_LOCK_DEG);

                if (aimLocked) {
                    turn = 0.0;
                    aimed = true;
                } else {
                    // Positive tx = target right of center, so turn right (negative turn)
                    turn = Math.max(-AIM_MAX_TURN, Math.min(AIM_MAX_TURN, -AIM_KP * txFiltered));
                }
            } else {
                aimLocked = false;
            }
        } else {
            aimLocked = false;
            txValid = false;
        }

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                turn,
                follower.pose().heading() - headingOffset
        );
        follower.manual(powers);
        follower.update();

        // ==========================================
        // GAMEPAD 2: AUXILIARY INTAKE SYSTEMS
        // ==========================================
        if (gamepad2.right_trigger > 0.1) {
            intake.setPower(gamepad2.right_trigger);
        } else if (gamepad2.left_trigger > 0.1) {
            intake.setPower(-gamepad2.left_trigger);
        } else {
            intake.setPower(0.0);
        }

        // ==========================================
        // GAMEPAD 2: SHOOTER & FLYWHEEL CONTROL
        // ==========================================
        // Toggle the flywheel on/off with Circle/B
        if (gamepad2.circle && !cross2Last) {
            isShooterActive = !isShooterActive;
            if (!isShooterActive) {
                shooter.setFlywheelPower(0.0);
            }
        }
        cross2Last = gamepad2.circle;

        // While on, set power every loop from the distance to the hive
        if (isShooterActive) {
            double dist = vision.distanceToTagsInches(tags);
            if (Double.isNaN(dist)) {
                double hiveX = (alliance == Alliance.RED) ? RED_HIVE_X : BLUE_HIVE_X;
                double hiveY = (alliance == Alliance.RED) ? RED_HIVE_Y : BLUE_HIVE_Y;
                dist = Math.hypot(hiveX - follower.pose().x(), hiveY - follower.pose().y());
            }
            shooter.setFlywheelPower(powerForDistance(dist));
        }

        // Shoot (Right Bumper) only when the driver is aimed (left bumper held and locked)
        if (gamepad2.right_bumper && !rightBumper2Last && aimed) {
            Scheduler.schedule(shooter.shootMacro());
        }
        rightBumper2Last = gamepad2.right_bumper;

        // ==========================================
        // PERSISTENT PROCESSING ENGINE
        // ==========================================
        Scheduler.execute();

        // Telemetry diagnostics
        telemetry.addData("X Position", follower.pose().x());
        telemetry.addData("Y Position", follower.pose().y());
        telemetry.addData("Robot Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Flywheel Status", isShooterActive ? "ENGAGED" : "OFF");
        telemetry.addData("Aim", gamepad1.left_bumper ? (aimed ? "LOCKED" : (txValid ? "AIMING" : "NO TAG")) : "off");
        telemetry.addData("Tag tx raw / filtered", String.format("%.2f / %.2f", vision.averageTx(tags), txFiltered));
        telemetry.addData("Shot distance (in)", vision.distanceToTagsInches(tags));
        telemetry.update();
    }

    private static double powerForDistance(double d) {
        if (d <= DIST_IN[0]) return POWER[0];
        if (d >= DIST_IN[DIST_IN.length - 1]) return POWER[POWER.length - 1];
        for (int i = 0; i < DIST_IN.length - 1; i++) {
            if (d <= DIST_IN[i + 1]) {
                double t = (d - DIST_IN[i]) / (DIST_IN[i + 1] - DIST_IN[i]);
                return POWER[i] + t * (POWER[i + 1] - POWER[i]);
            }
        }
        return POWER[POWER.length - 1];
    }

    private int[] hiveTags() {
        return alliance == Alliance.RED ? VisionSubsystem.RED_HIVE_TAGS : VisionSubsystem.BLUE_HIVE_TAGS;
    }
}

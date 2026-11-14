package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterIndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous(name = "Ivy Pedro Sequential Shooter Auto V3", group = "Autonomous")
public class PedroIvyLimelightAutoV3 extends OpMode {

    private Follower follower;
    private VisionSubsystem vision;
    private IntakeSubsystem intake;
    private ShooterIndexerSubsystem shooter;

    private Path approachPath;
    private Path parkEndPath;

    // --- Alliance selection ---
    private enum Alliance { RED, BLUE }
    private Alliance currentAlliance = Alliance.RED;
    private boolean lastGamepad1X = false;
    private boolean lastGamepad1B = false;

    // --- Red route (field inches, corner origin, red frame) ---
    // Step 1: start, and the shooting area
    private final Pose redStartPose = new Pose(53.3, 6.5, Math.toRadians(270));
    private final Pose redTargetAreaPose = new Pose(53.36, 17, Math.toRadians(270));

    // Step 4: curve around the Red HIVE (x 46 to 48.6, y 50 to 91.4) to the park end
    private final Pose redParkCtrl1 = new Pose(15.58, 32.54, 0);
    private final Pose redParkCtrl2 = new Pose(27.25, 82.94, 0);
    private final Pose redParkEndPose = new Pose(14.10, 90.93, Math.toRadians(329));

    // Active poses, selected in start()
    private Pose startPose;
    private Pose targetAreaPose;
    private Pose parkCtrl1;
    private Pose parkCtrl2;
    private Pose parkEndPose;

    // --- Hive targeting ---
    // Measure on the field. Zeros mean unmeasured, so the shooter uses a fixed power.
    private static final double RED_HIVE_X = 58.0, RED_HIVE_Y = 71.5;
    private static final double BLUE_HIVE_X = 84.0, BLUE_HIVE_Y = 71.5;
    private static final double SHOOT_POWER_FALLBACK = 0.65;

    private int[] targetTags = VisionSubsystem.RED_HIVE_TAGS;
    private double hiveX, hiveY;

    // --- Alignment state ---
    private long alignStartMs = 0;
    private boolean turnIssued = false;
    private Command turnCommand = null;

    @Override
    public void init() {
        Scheduler.reset();
        OpModeStorage.autonomousEndPose = null; // never hand a stale pose to TeleOp

        follower = Constants.create(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
        shooter = new ShooterIndexerSubsystem(hardwareMap);
        vision.switchPipeline(0);

        telemetry.addData("Status", "Initialized. Select Alliance on Gamepad1.");
    }

    @Override
    public void init_loop() {
        if (gamepad1.b && !lastGamepad1B) currentAlliance = Alliance.RED;
        if (gamepad1.x && !lastGamepad1X) currentAlliance = Alliance.BLUE;
        lastGamepad1B = gamepad1.b;
        lastGamepad1X = gamepad1.x;

        telemetry.addLine("=== ALLIANCE CONFIGURATION ===");
        telemetry.addData("Selected Alliance", currentAlliance == Alliance.RED ? "RED (B)" : "BLUE (X)");
        telemetry.addData("Press B for red, X for blue", "");
        telemetry.update();
    }

    @Override
    public void start() {
        vision.startCamera();
        OpModeStorage.isRedAlliance = (currentAlliance == Alliance.RED);
        boolean red = currentAlliance == Alliance.RED;

        if (red) {
            startPose = redStartPose;
            targetAreaPose = redTargetAreaPose;
            parkCtrl1 = redParkCtrl1;
            parkCtrl2 = redParkCtrl2;
            parkEndPose = redParkEndPose;
            targetTags = VisionSubsystem.RED_HIVE_TAGS;
            hiveX = RED_HIVE_X;
            hiveY = RED_HIVE_Y;
        } else {
            startPose = toBlue(redStartPose);
            targetAreaPose = toBlue(redTargetAreaPose);
            parkCtrl1 = toBlue(redParkCtrl1);
            parkCtrl2 = toBlue(redParkCtrl2);
            parkEndPose = toBlue(redParkEndPose);
            targetTags = VisionSubsystem.BLUE_HIVE_TAGS;
            hiveX = BLUE_HIVE_X;
            hiveY = BLUE_HIVE_Y;
        }

        follower.setPose(startPose);

        // Step 1: start to shooting area
        Command driveToZone = Command.build()
                .setStart(() -> {
                    approachPath = line(startPose, targetAreaPose).linear(startPose, targetAreaPose);
                    follower.follow(approachPath);
                })
                .setDone(() -> !follower.isBusy());

        // Step 2: turn toward the average bearing of the hive tags
        Command alignToHive = Command.build()
                .setStart(() -> {
                    alignStartMs = System.currentTimeMillis();
                    turnIssued = false;
                    turnCommand = null;

                    double tx = vision.averageTx(targetTags);
                    if (Double.isNaN(tx) || Math.abs(tx) <= 1.0) {
                        turnIssued = true; // no tags or already aimed
                        return;
                    }
                    Pose here = follower.pose();
                    // Positive tx = target right of center, so subtract to turn right
                    double h = normalizeAngle(here.heading() - Math.toRadians(tx));
                    turnCommand = hold(follower, new Pose(here.x(), here.y(), h));
                    Scheduler.schedule(turnCommand);
                    turnIssued = true;
                })
                .setDone(() -> (turnIssued && (turnCommand == null || !follower.isBusy()))
                        || System.currentTimeMillis() - alignStartMs > 3000)
                .setEnd(end -> telemetry.addLine("Alignment done or timed out."));

        // Step 3: set flywheel power from distance to the hive, then feed
        Command setShotPower = instant(() -> {
            boolean hiveMeasured = hiveX != 0.0 || hiveY != 0.0;
            double dist = vision.distanceToTagsInches(targetTags);
            if (Double.isNaN(dist) && hiveMeasured) {
                dist = Math.hypot(hiveX - follower.pose().x(), hiveY - follower.pose().y());
            }
            double power = Double.isNaN(dist)
                    ? SHOOT_POWER_FALLBACK
                    : ShooterIndexerSubsystem.powerForDistance(dist);
            shooter.setFlywheelPower(power);
        });

        Command shootAllBalls = sequential(
                setShotPower,
                waitMs(800),
                shooter.feedPollen(),
                waitMs(3000),
                shooter.stopIndexer(),
                shooter.stopFlywheel()
        );

        // Step 4: curve around the hive to the park end point
        Command driveToParkEnd = Command.build()
                .setStart(() -> {
                    Pose s = follower.pose();
                    parkEndPath = curve(s, parkCtrl1, parkCtrl2, parkEndPose).linear(s, parkEndPose);
                    follower.follow(parkEndPath);
                })
                .setDone(() -> !follower.isBusy());

        // Step 5: store the final pose for TeleOp
        Command saveFinalPose = instant(() -> OpModeStorage.autonomousEndPose = follower.pose());

        Scheduler.schedule(
                sequential(
                        driveToZone,
                        alignToHive,
                        shootAllBalls,
                        driveToParkEnd,
                        saveFinalPose
                )
        );
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        // Keep the TeleOp handoff current even if AUTO is stopped early
        OpModeStorage.autonomousEndPose = follower.pose();

        telemetry.addData("Alliance Status", currentAlliance == Alliance.RED ? "RED" : "BLUE");
        telemetry.addData("Robot Pose", follower.pose().toString());
        telemetry.update();
    }

    // BIOBUZZ field is rotated 180 degrees between alliances, not mirrored
    private static Pose toBlue(Pose p) {
        return new Pose(144.0 - p.x(), 144.0 - p.y(), normalizeAngle(p.heading() + Math.PI));
    }

    private static double normalizeAngle(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }
}

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;


public class ShooterIndexerSubsystem {
    private final DcMotorEx flywheel;
    private final CRServo indexer;

    // Constants for your BioBuzz setup
    private static final double SHOOTING_POWER = 0.65;
    private static final double INDEXER_FEED_POWER = 1.0;
    private static final double INDEXER_STOP_POWER = 0.0;

    public ShooterIndexerSubsystem(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        indexer = hardwareMap.get(CRServo.class, "Indexer"); // Your CR Servo indexer
    }

    public Command runFlywheel(double power) {
        return Command.build()
                .setStart(() -> flywheel.setPower(power))
                .setEnd(endCondition -> flywheel.setPower(0.0));
    }

    // Command to rev up the shooter flywheel
    public Command spinUp() {
        return Command.build()
                .setStart(() -> flywheel.setPower(SHOOTING_POWER))
                .setDone(() -> true) // Finishes instantly, letting the motor rev in the background
                .requiring(flywheel);
    }

    // Command to stop the flywheel
    public Command stopFlywheel() {
        return Command.build()
                .setStart(() -> flywheel.setPower(0))
                .setDone(() -> true)
                .requiring(flywheel);
    }

    // Distance (inches) to hive -> flywheel power. Placeholder values until calibrated on the field.
    private static final double[] DIST_IN = {24, 42, 50, 55};
    private static final double[] POWER   = {0.44, 0.54, 0.64, 0.74};

    public static double powerForDistance(double d) {
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


    // Direct power control for the flywheel, used by teleop each loop
    public void setFlywheelPower(double power) {
        flywheel.setPower(power);
    }

    // Command to feed pollen via the CR Servo Indexer
    public Command feedPollen() {
        return Command.build()
                .setStart(() -> indexer.setPower(-INDEXER_FEED_POWER))
                .setDone(() -> true)
                .requiring(indexer);
    }

    // Command to stop the CR Servo Indexer
    public Command stopIndexer() {
        return Command.build()
                .setStart(() -> indexer.setPower(INDEXER_STOP_POWER))
                .setDone(() -> true)
                .requiring(indexer);
    }

    // A unified macro command to execute a single cycle shot
    public Command fireShot() {
        return sequential(
                spinUp(),
                waitMs(1000),       // Allow time for the flywheel to reach target velocity
                feedPollen(),
                waitMs(500),       // Time duration required to push item completely through
                stopIndexer()

        );
    }

    public Command shootMacro() {
        return sequential(
                spinUp(),
                waitMs(1000),       // Allow time for the flywheel to reach target velocity
                feedPollen(),
                waitMs(500),       // Time duration required to push item completely through
                stopIndexer()
        );
    }
}

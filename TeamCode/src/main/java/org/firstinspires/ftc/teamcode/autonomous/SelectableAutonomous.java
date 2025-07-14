package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.SelectableAutonomous.drawCurrent;
import static org.firstinspires.ftc.teamcode.autonomous.SelectableAutonomous.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.autonomous.SelectableAutonomous.drivetrain;
import static org.firstinspires.ftc.teamcode.autonomous.SelectableAutonomous.elevator;
import static org.firstinspires.ftc.teamcode.autonomous.SelectableAutonomous.intake;
import static org.firstinspires.ftc.teamcode.autonomous.SelectableAutonomous.manipulator;
import static org.firstinspires.ftc.teamcode.autonomous.SelectableAutonomous.pathChain;
import static org.firstinspires.ftc.teamcode.autonomous.SelectableAutonomous.telemetryManager;

import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.configurables.ConfigurablesManager;
import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;
import com.bylazar.ftcontrol.panels.configurables.annotations.IgnoreConfigurable;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.autonomous.paths.Trajectories;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Manipulator;
import org.firstinspires.ftc.teamcode.utilities.constansts.DrivetrainConstants;

import java.util.ArrayList;
import java.util.List;

@Configurable
@TeleOp(name = "Autonomous", group = "Auto")
public class SelectableAutonomous extends SelectableOpMode {
    public static Drivetrain drivetrain;
    public static Elevator elevator;
    public static Intake intake;
    public static Manipulator manipulator;

    public static PathChain pathChain;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    public SelectableAutonomous() {
        super("Select an Autonomous", s -> {
            s.folder("Bucket", l -> {
                l.add("0+4 No Park", FourBucketAutonomousNoPark::new);
                l.add("0+4 Park", FourBucketAutonomousNoPark::new);
                l.add("0+5 No Sub", FourBucketAutonomousNoPark::new);
                l.add("0+6", FourBucketAutonomousNoPark::new);
            });
            s.folder("Speciman", p -> {
                p.add("4+0", FourBucketAutonomousNoPark::new);
                p.add("5+0 Preload", FourBucketAutonomousNoPark::new);
                p.add("5+0 No-Preload", FourBucketAutonomousNoPark::new);
                p.add("6+0", FourBucketAutonomousNoPark::new);
            });
        });
    }

    @Override
    public void onSelect() {
        if (drivetrain == null || elevator == null || intake == null || manipulator == null) {
            drivetrain = new Drivetrain(hardwareMap, telemetry);
            elevator = new Elevator(hardwareMap, telemetry);
            intake = new Intake(hardwareMap, telemetry);
            manipulator = new Manipulator(hardwareMap, telemetry);

            ConfigurablesManager.INSTANCE.init(hardwareMap.appContext);
        }

        telemetryManager = Panels.getTelemetry();
    }

    @Override
    public void onLog(List<String> lines) {
        Panels.getTelemetry().debug(lines.toArray(new String[0]));
        Panels.getTelemetry().update();
    }

    public static void drawCurrent() {
        drivetrain.drawCurrentTrajectory();
    }

    public static void drawCurrentAndHistory() {
        drivetrain.drawCurrentAndHistoricalTrajectory();
    }

    /** This creates a full stop of the robot by setting the drive motors to run at 0 power. */
    public static void stopRobot() {
        drivetrain.follower.startTeleopDrive(true);
        drivetrain.follower.setTeleOpDrive(0,0,0,true);
    }
}

class FourBucketAutonomousNoPark extends CommandOpMode {
    @Override
    public void initialize() {
        drivetrain.follower.setStartingPose(new Pose(
                DrivetrainConstants.allBucketStartingPose.getX(),
                DrivetrainConstants.allBucketStartingPose.getY(),
                DrivetrainConstants.allBucketStartingPose.getRotation().getDegrees()
        ));

        pathChain = Trajectories.fourBucketNoPark;

        schedule(
                new SequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        new FollowTrajectoryCommand(drivetrain, pathChain.getPath(0), true).setCompletionThreshold(0.975)
                )
        );
    }

    @Override
    public void runOpMode() {
        initialize();

        while (!opModeIsActive()) {
            manipulator.onInit();
            elevator.onInit();
            intake.onInit();
        }

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }

        reset();
    }
}

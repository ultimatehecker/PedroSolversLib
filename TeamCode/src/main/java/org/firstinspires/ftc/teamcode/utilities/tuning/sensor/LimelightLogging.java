package org.firstinspires.ftc.teamcode.utilities.tuning.sensor;

import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.configurables.annotations.IgnoreConfigurable;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.utilities.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utilities.geometry.Rotation2d;

@TeleOp(name="LimelightLogging")
public class LimelightLogging extends OpMode {
    private Limelight3A limelight3A;
    private boolean isTargetFound, isLocked;
    private Pose2d targetLocation;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.setPollRateHz(100);
        limelight3A.start();
        telemetry.setMsTransmissionInterval(11);
        limelight3A.pipelineSwitch(1);
        telemetryManager = Panels.getTelemetry();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        LLResult llResult = limelight3A.getLatestResult();
        if(llResult != null) {
            telemetry.addData("something", limelight3A.getLatestResult().getPythonOutput().length);
            telemetry.addData("something1", limelight3A.getLatestResult().getTy());
            telemetry.addData("something2", limelight3A.getLatestResult().getTx());
            telemetry.addData("something3", limelight3A.getLatestResult().getTa());
            telemetry.addData("something4", limelight3A.getLatestResult().getTargetingLatency());
            telemetry.addData("something5", limelight3A.getTimeSinceLastUpdate());
            telemetry.addData("something6", limelight3A.getLatestResult().getTxNC());

            long staleness = llResult.getStaleness();
            if (staleness < 100) { // Less than 100 milliseconds old
                telemetry.addData("Data", "Good");
            } else {
                telemetry.addData("Data", "Old (" + staleness + " ms)");
            }
        }

        telemetry.addData("LLResult Validity: ", limelight3A.getLatestResult().isValid());
        telemetryManager.update(telemetry);
    }
}

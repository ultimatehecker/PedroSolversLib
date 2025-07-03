package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.configurables.annotations.IgnoreConfigurable;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Vision extends SubsystemBase {
    private Limelight3A limelight;
    private Telemetry telemetry;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    public Vision(HardwareMap aHardwareMap, Telemetry telemetry) {
        limelight = aHardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        telemetryManager = Panels.getTelemetry();
        this.telemetry = telemetry;
    }

    public double getTx() {
        LLResult limelightResult = limelight.getLatestResult();
        if(limelightResult != null) {
            if(limelightResult.isValid()) {
                return limelightResult.getTx();
            } else {
                return -100;
            }
        } else {
            return -100;
        }
    }

    public double getTy() {
        LLResult limelightResult = limelight.getLatestResult();
        if(limelightResult != null) {
            if(limelightResult.isValid()) {
                return limelightResult.getTy();
            } else {
                return -100;
            }
        } else {
            return -100;
        }
    }

    @Override
    public void periodic() {
        telemetryManager.debug("LL Detector tX: " + getTx());
        telemetryManager.debug("LL Detector tY: " + getTy());
    }
}

package org.firstinspires.ftc.teamcode.utilities.tuning.sensor;

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

    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
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

        if(llResult != null && llResult.isValid()) {
            if(!llResult.getDetectorResults().isEmpty() && !isLocked) {
                targetLocation = new Pose2d(llResult.getTx(), llResult.getTy(), new Rotation2d());
                isLocked = true;
            }
        }

        if(isLocked && targetLocation != null) {
            PIDFController xController = new PIDFController(0,0,0,0);
            PIDFController yController = new PIDFController(0,0,0,0);
        }

        telemetry.addData("Target Locked", isLocked);
        telemetry.addData("Target X", targetLocation != null ? targetLocation.getX() : "N/A");
        telemetry.addData("Target Y", targetLocation != null ? targetLocation.getY() : "N/A");
    }
}

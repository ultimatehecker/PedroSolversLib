package org.firstinspires.ftc.teamcode.utilities;

import com.pedropathing.telemetry.SelectScope;
import com.pedropathing.telemetry.Selector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

public abstract class SelectableAutonomousMode extends CommandOpMode {
    private final Selector<Supplier<CommandOpMode>> selector;
    private CommandOpMode selectedOpMode;
    private final static String[] MESSAGE = {
            "Use the d-pad to move the cursor.",
            "Press right bumper to select.",
            "Press left bumper to go back."
    };

    public SelectableAutonomousMode(String name, Consumer<SelectScope<Supplier<CommandOpMode>>> opModes) {
        selector = Selector.create(name, opModes, MESSAGE);
        selector.onSelect(opModeSupplier -> {
            onSelect();
            selectedOpMode = opModeSupplier.get();
            selectedOpMode.gamepad1 = gamepad1;
            selectedOpMode.gamepad2 = gamepad2;
            selectedOpMode.telemetry = telemetry;
            selectedOpMode.hardwareMap = hardwareMap;
            selectedOpMode.initialize();
        });
    }

    protected void onSelect() {
    }

    protected void onLog(List<String> line) {
    }

    @Override
    public final void initialize() {
    }

    @Override
    public final void initialize_loop() {
        if (selectedOpMode == null) {
            if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed())
                selector.decrementSelected();
            else if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed())
                selector.incrementSelected();
            else if (gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed())
                selector.select();
            else if (gamepad1.leftBumperWasPressed() || gamepad2.leftBumperWasPressed())
                selector.goBack();

            List<String> lines = selector.getLines();
            for (String line : lines) {
                telemetry.addLine(line);
            }
            onLog(lines);
        } else selectedOpMode.initialize_loop();
    }

    /*

    @Override
    public final void start() {
        if (selectedOpMode == null) throw new RuntimeException("No OpMode selected!");
        selectedOpMode.start();
    }

     */

    @Override
    public final void run() {
        selectedOpMode.run();
    }

    @Override
    public final void end() {
        if (selectedOpMode != null) selectedOpMode.end();
    }
}
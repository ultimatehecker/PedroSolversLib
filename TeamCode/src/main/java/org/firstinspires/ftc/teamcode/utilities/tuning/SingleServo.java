package org.firstinspires.ftc.teamcode.utilities.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.solversHardware.SolversServo;

@TeleOp(name="TestServo")
public class SingleServo extends OpMode {
    private SolversServo rightLinkageServo;

    @Override
    public void init() {
        rightLinkageServo = new SolversServo(hardwareMap.get(Servo.class, "linkageServo2"), 0.01);
        rightLinkageServo.setDirection(Servo.Direction.FORWARD);
    }


    @Override
    public void loop() {
        rightLinkageServo.setPosition(0.95);
    }
}

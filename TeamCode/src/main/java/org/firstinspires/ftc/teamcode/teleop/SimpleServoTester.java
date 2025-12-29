package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Map;

@TeleOp(name = "Simple Servo Tester", group = "Toolbox")
public class SimpleServoTester extends OpMode {
    // configuration
    private static final double POS_DELTA = 0.02;

    // servo data
    private Servo servo;
    private String[] servoChoices;
    private int selectedServoIndex = 0;

    public enum Mode {
        SELECTING,
        TESTING
    };

    // state
    private Mode mode = Mode.SELECTING;

    // debounce
    private boolean xWasPressed = false;
    private boolean dpadUpWasPressed = false;
    private boolean dpadDownWasPressed = false;

    @Override
    public void init() {
        servoChoices = hardwareMap.servo.entrySet().stream()
                .map(Map.Entry::getKey)
                .toArray(String[]::new);

        if (servoChoices.length == 0) {
            telemetry.addLine("ERROR: No servos found in hardware map!");
        }
    }

    @Override
    public void init_loop() {
        if (servoChoices.length == 0) return;

        // cycle to the next servo
        boolean xIsPressed = gamepad1.x;
        if (xIsPressed && !xWasPressed) {
            selectedServoIndex = (selectedServoIndex + 1) % servoChoices.length;
        }
        xWasPressed = xIsPressed;

        telemetry.addData("Selected Servo", servoChoices[selectedServoIndex]);
        telemetry.addLine("Press X to select next servo");
        telemetry.addLine("Press START when ready to test");
    }

    @Override
    public void start() {
        if (servoChoices.length == 0) return;

        // move into testing mode
        mode = Mode.TESTING;
        String name = servoChoices[selectedServoIndex];
        servo = hardwareMap.servo.get(name);
    }

    @Override
    public void loop() {
        if (mode != Mode.TESTING || servo == null) return;

        boolean dpadUpIsPressed = gamepad1.dpad_up;
        if (dpadUpIsPressed && !dpadUpWasPressed) {
            servo.setPosition(servo.getPosition() + POS_DELTA);
        }
        dpadUpWasPressed = dpadUpIsPressed;

        boolean dpadDownIsPressed = gamepad1.dpad_down;
        if (dpadDownIsPressed && !dpadDownWasPressed) {
            servo.setPosition(servo.getPosition() - POS_DELTA);
        }
        dpadDownWasPressed = dpadDownIsPressed;

        telemetry.addLine("*** Servo Control ***");
        telemetry.addData("Dpad Up/Down Buttons", "Increment/Decrement Position (%.3f)", POS_DELTA);

        telemetry.addLine("*** Servo Data ***");
        telemetry.addData("Name", servoChoices[selectedServoIndex]);
        telemetry.addData("Position", "%.4f", servo.getPosition());
    }
}

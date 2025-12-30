package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Map;

@TeleOp(name = "Simple Motor Tester", group = "Toolbox")
public class SimpleMotorTester extends OpMode {
    // motor data
    private DcMotor motor;
    private String[] motorChoices;
    private int selectedMotorIndex = 0;

    public enum Mode {
        SELECTING,
        TESTING
    };

    // state
    private Mode mode = Mode.SELECTING;

    // debounce
    private boolean xWasPressed = false;

    @Override
    public void init() {
        motorChoices = hardwareMap.dcMotor.entrySet().stream()
                .map(Map.Entry::getKey)
                .toArray(String[]::new);

        if (motorChoices.length == 0) {
            telemetry.addLine("ERROR: No motors found in hardware map!");
        }
    }

    @Override
    public void init_loop() {
        if (motorChoices.length == 0) return;

        // cycle to the next motor
        boolean xIsPressed = gamepad1.x;
        if (xIsPressed && !xWasPressed) {
            selectedMotorIndex = (selectedMotorIndex + 1) % motorChoices.length;
        }
        xWasPressed = xIsPressed;

        telemetry.addData("Selected Motor", motorChoices[selectedMotorIndex]);
        telemetry.addLine("Press X to select next motor");
        telemetry.addLine("Press START when ready to test");
    }

    @Override
    public void start() {
        if (motorChoices.length == 0) return;

        // move into testing mode
        mode = Mode.TESTING;
        String name = motorChoices[selectedMotorIndex];
        motor = hardwareMap.dcMotor.get(name);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        if (mode != Mode.TESTING || motor == null) return;

        double power = -gamepad1.left_stick_y;
        motor.setPower(power);

        telemetry.addLine("*** Motor Control ***");
        telemetry.addData("Left Stick", "Motor Power (%.2f)", power);

        telemetry.addLine("\n*** Motor Data ***");
        telemetry.addData("Name", motorChoices[selectedMotorIndex]);
    }
}
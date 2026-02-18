package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import team.techtigers.core.display.AdafruitNeoPixel;
import team.techtigers.core.display.Color;

public class Display {
    private AdafruitNeoPixel neoPixel;
    private static int LEDS_PER_RING = 24;

    public Display(LinearOpMode opMode) {
        neoPixel = opMode.hardwareMap.get(AdafruitNeoPixel.class, "display");
        neoPixel.initialize(LEDS_PER_RING * 3, 3);
    }

    public void periodic(boolean ringOne, boolean ringTwo, boolean ringThree) {
        Color[] ledColors = new Color[LEDS_PER_RING * 3];
        for (int i = 0; i < LEDS_PER_RING * 3; i++) {
            if (i >= 0 && i < LEDS_PER_RING) {
                ledColors[i] = ringOne ? Color.GREEN : Color.BLACK;
            }
            if (i >= LEDS_PER_RING && i < LEDS_PER_RING * 2) {
                ledColors[i] = ringTwo ? Color.GREEN : Color.BLACK;
            }
            if (i >= LEDS_PER_RING * 2 && i < LEDS_PER_RING * 3) {
                ledColors[i] = ringThree ? Color.GREEN : Color.BLACK;
            }
        }
        neoPixel.setLeds(0, ledColors);
    }
}

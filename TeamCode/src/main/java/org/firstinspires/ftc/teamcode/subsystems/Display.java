package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Arrays;

import team.techtigers.core.display.AdafruitNeoPixel;
import team.techtigers.core.display.Color;

public class Display {
    private final AdafruitNeoPixel neoPixel;
    private static final int LEDS_PER_RING = 24;
    private static final int TOTAL_RINGS = 3;

    private Color[] leds = new Color[LEDS_PER_RING * TOTAL_RINGS];

    public static final Color COLOR_SAFE = Color.GREEN;
    public static final Color COLOR_OVER = Color.RED;

    public Display(LinearOpMode opMode) {
        neoPixel = opMode.hardwareMap.get(AdafruitNeoPixel.class, "display");

        neoPixel.initialize(leds.length, 3);

        Arrays.fill(leds, Color.BLACK);
    }

    public void periodic(Intake.PossessionState possessionState) {
        int ringsToLight = 0;
        Color activeColor = COLOR_SAFE;

        switch (possessionState) {
            case NONE:
                ringsToLight = 0;
                activeColor = Color.BLACK;
                break;

            case ONE:
                ringsToLight = 1;
                break;

            case TWO:
                ringsToLight = 2;
                break;

            case THREE:
                ringsToLight = 3;
                break;

            case OVER:
                ringsToLight = 3;
                activeColor = COLOR_OVER;
                break;
        }

        updateLeds(ringsToLight, activeColor);
    }

    private void updateLeds(int numRings, Color color) {
        int ledThreshold = numRings * LEDS_PER_RING;

        for (int i = 0; i < leds.length; i++) {
            leds[i] = (i < ledThreshold) ? color : Color.BLACK;
        }

        neoPixel.setLeds(0, leds);
    }
}
package org.firstinspires.ftc.teamcode.util;

import java.util.stream.Stream;

public class Selector {
    private int selected = 0;
    private String[] choices;

    public Selector(Stream<String> choices) {
        this.choices = choices.toArray(String[]::new);
    }

    public Selector(Stream<String> choices, String[] otherChoices) {
        String[] choice = choices.toArray(String[]::new);

        this.choices = new String[choice.length+otherChoices.length];

        System.arraycopy(choice, 0, this.choices, 0, choice.length);
        System.arraycopy(otherChoices, 0, this.choices, choice.length, otherChoices.length);
    }

    public void selectNext() {
        selected = (selected + 1) % choices.length;
    }

    public String selected() {
        return choices[selected];
    }

    public Stream<String> allChoices() {
        return Stream.of(choices);
    }
}
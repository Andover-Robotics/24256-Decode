package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Vector2d;

public class SimplePosition {
    public double x;
    public double y;

    public SimplePosition(int x, int y) {
        this.x = x;
        this.y = y;
    }

    public Vector2d toVector() {
        return new Vector2d(x, y);
    }
}

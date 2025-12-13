package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;

public class Goal {
    Vector2d x0;
    Vector2d x1;
    Vector2d x2;

    public Goal(Vector2d x0, Vector2d x1, Vector2d x2) {
        this.x0 = x0;
        this.x1 = x1;
        this.x2 = x2;
    }

    private static double cross(Vector2d p0, Vector2d p1) {
        return p0.x * p1.y - p0.y * p1.x;
    }

    private static Vector2d raySegmentIntersect(Pose2d ray, Vector2d p0, Vector2d p1) {
        final double EPSILON = 1e-6;

        Vector2d rayOrigin = ray.position;
        Vector2d rayDirection = new Vector2d(Math.cos(ray.heading.log()), Math.sin(ray.heading.log()));

        Vector2d edge = p1.minus(p0);
        Vector2d diff = p0.minus(rayOrigin);

        double denominator = cross(rayDirection, edge);

        if (Math.abs(denominator) < EPSILON) {
            return null;
        }

        double t = cross(diff, edge) / denominator;
        double u = cross(diff, rayDirection) / denominator;

        if (t > EPSILON && u >= -EPSILON && u <= 1.0 + EPSILON) { /* hit in bounds */
            return p0.plus(p1.minus(p0).times(t));
        }

        return null;
    }

    public ArrayList<Vector2d> getHits(Pose2d ray) {
        ArrayList<Vector2d> hits = new ArrayList<Vector2d>();

        Vector2d intersection0 = raySegmentIntersect(ray, x0, x1);
        if (intersection0 != null) hits.add(intersection0);

        Vector2d intersection1 = raySegmentIntersect(ray, x1, x2);
        if (intersection1 != null) hits.add(intersection1);

        Vector2d intersection2 = raySegmentIntersect(ray, x2, x0);
        if (intersection2 != null) hits.add(intersection2);

        return hits;
    }

    public Vector2d getGoal(Pose2d ray) {
        ArrayList<Vector2d> hits = getHits(ray);

        if (hits.isEmpty()) {
            return null;
        } else if (hits.size() == 1) {
            return hits.get(0);
        } else {
            return hits.get(0).plus(hits.get(1)).div(2); // midpoint
        }
    }
}

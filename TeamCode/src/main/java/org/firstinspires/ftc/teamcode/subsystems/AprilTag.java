package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTag {
    private VisionPortal visionPortal;
    private AprilTagProcessor processor;

    private AprilTagType obelisk = null;
    private AprilTagResult goal = null;
    private AprilTagType colorTarget;

    private static double redGoalX = 0;
    private static double redGoalY = 0;

    private static double blueGoalX = 0;
    private static double blueGoalY = 0;

    // reference: ConceptAprilTagLocalization.java
    private static Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private static YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    public AprilTag(HardwareMap hardwareMap) {
        processor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"), processor);
        if (Bot.alliance == Bot.Alliance.RED) {
            colorTarget = AprilTagType.GOAL_RED;
        } else if (Bot.alliance == Bot.Alliance.BLUE) {
            colorTarget = AprilTagType.GOAL_BLUE;
        }
    }

    public AprilTagType getColorTarget() {
        return colorTarget;
    }

    public void setColorTarget(AprilTagType colorTarget) {
        this.colorTarget = colorTarget;
    }

    public enum AprilTagType {
        // source: https://ftc-resources.firstinspires.org/ftc/field/apriltag-art
        GOAL_RED(24),
        GOAL_BLUE(20),
        OBELISK_GPP(21),
        OBELISK_PGP(22),
        OBELISK_PPG(23);

        private final int id;

        AprilTagType(int id) {
            this.id = id;
        }

        public static AprilTagType fromAprilTagId(int id) {
            for (AprilTagType e : AprilTagType.values()) {
                if (e.id == id) {
                    return e;
                }
            }

            throw new IllegalArgumentException();
        }
    }

    public class AprilTagResult {
        AprilTagType type;
        Pose3D robotPose;

        public AprilTagResult(AprilTagType type, Pose3D robotPose) {
            this.type = type;
            this.robotPose = robotPose;
        }
    }

    public void updateDetections() {
        goal = null;

        for (AprilTagDetection detection : processor.getDetections()) {
            if (detection.metadata == null)
                continue;

            AprilTagType type = AprilTagType.fromAprilTagId(detection.id);
            AprilTagResult result = new AprilTagResult(
                    type,
                    detection.robotPose
            );

            if (type == colorTarget)
                goal = result;

            if (type == AprilTagType.OBELISK_GPP || type == AprilTagType.OBELISK_PGP || type == AprilTagType.OBELISK_PPG)
                obelisk = type;
        }
    }

    public AprilTagResult getGoal() {
        return goal;
    }

    public AprilTagType getObelisk() {
        return obelisk;
    }

    public Double getDistance() {
        if (goal == null) {
            return null;
        } else {
            Position robotPosition = goal.robotPose.getPosition();
            double goalX = (Bot.alliance == Bot.Alliance.RED) ? redGoalX : blueGoalX;
            double goalY = (Bot.alliance == Bot.Alliance.RED) ? redGoalY : blueGoalY;
            return Math.hypot(goalX - robotPosition.x, goalY - robotPosition.y);
        }
    }
}

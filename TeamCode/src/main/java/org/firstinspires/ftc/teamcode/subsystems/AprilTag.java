package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

    private static double heightOffset = 0;

    public AprilTag(HardwareMap hardwareMap) {
        processor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
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
        AprilTagPoseFtc ftcPose;

        public AprilTagResult(AprilTagType type, AprilTagPoseFtc ftcPose) {
            this.type = type;
            this.ftcPose = ftcPose;
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
                    detection.ftcPose
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
            double aprilTagDistance = goal.ftcPose.range;
            double twoDim = Math.sqrt(aprilTagDistance * aprilTagDistance - heightOffset * heightOffset);
            double bearingCorrection = twoDim * Math.cos(goal.ftcPose.bearing); // cos(x) = cos(-x)
            return bearingCorrection;
        }
    }
}

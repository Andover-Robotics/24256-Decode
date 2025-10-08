package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTag {
    private VisionPortal visionPortal;
    private AprilTagProcessor processor;

    private AprilTagType obelisk = null;
    private AprilTagResult goal = null;
    private AprilTagType colorTarget;

    public AprilTag(HardwareMap hardwareMap, AprilTagType colorTarget) {
        processor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .build();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"), processor);
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
        double range;
        double bearing;
        double yaw;

        public AprilTagResult(AprilTagType type, double range, double bearing, double yaw) {
            this.type = type;
            this.range = range;
            this.bearing = bearing;
            this.yaw = yaw;
        }
    }

    public void updateDetections() {
        // obelisk only needs to be get once, but we should
        // always set goal to null to ensure that the user
        // knows if they're getting a consistent reading or not
        goal = null;

        for (AprilTagDetection detection : processor.getDetections()) {
            if (detection.metadata == null)
                continue;

            AprilTagType type = AprilTagType.fromAprilTagId(detection.id);
            AprilTagResult result = new AprilTagResult(
                    type,
                    detection.ftcPose.range,
                    detection.ftcPose.bearing,
                    detection.ftcPose.yaw
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
}

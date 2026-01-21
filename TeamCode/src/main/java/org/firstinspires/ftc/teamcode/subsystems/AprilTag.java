package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public class AprilTag {
    private VisionPortal visionPortal;
    private AprilTagProcessor processor;

    public AprilTagType obelisk = null;
    public AprilTagResult goal = null;
    private AprilTagType colorTarget;

    private ExposureControl exposureControl;
    private GainControl gainControl;

    public static double CAMERA_TIMEOUT = 5.0 * 1000;

    public AprilTag(LinearOpMode opMode) {
        processor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .build();
        visionPortal = VisionPortal.easyCreateWithDefaults(opMode.hardwareMap.get(WebcamName.class, "webcam"), processor);

        opMode.telemetry.addLine("Calibrating camera...");
        double beginTs = System.currentTimeMillis();
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING
                && System.currentTimeMillis() - beginTs < CAMERA_TIMEOUT) {
            opMode.sleep(1);
        }

        exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        gainControl = visionPortal.getCameraControl(GainControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(3, TimeUnit.MILLISECONDS);
        gainControl.setGain((int) (gainControl.getMaxGain() * 0.6));
        opMode.telemetry.clearAll();
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
        if (Bot.alliance == Bot.Alliance.RED) {
            colorTarget = AprilTagType.GOAL_RED;
        } else if (Bot.alliance == Bot.Alliance.BLUE) {
            colorTarget = AprilTagType.GOAL_BLUE;
        }

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
}
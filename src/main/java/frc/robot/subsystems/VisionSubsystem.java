package frc.robot.subsystems;

import static frc.robot.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {

    @FunctionalInterface
    public interface MeasurementConsumer {
        void accept(Pose2d pose, double fpgaSeconds, Matrix<N3, N1> standardDeviations);
    }

    public record AimObservation(double bearingRadians, double distanceMeters) {}

    private final PhotonCamera camera = new PhotonCamera(CAMERA_NAME);
    private final PhotonPoseEstimator estimator =
        new PhotonPoseEstimator(FIELD_LAYOUT, ROBOT_TO_CAMERA);
    private final MeasurementConsumer consumer;
    private final Supplier<Pose2d> robotPose;
    private final Field2d field = new Field2d();
    
    private PhotonPipelineResult latest;
    private double lastFrameTime = Double.NEGATIVE_INFINITY;
    private int passedCount;
    private int rejectedCount;
    private int fusedCount;
    private String lastReason = "No frame yet";

public VisionSubsystem(String cameraName, Transform3d robotToCamera, MeasurementConsumer consumer, Supplier<Pose2d> robotPose) {
    this.consumer = consumer;
    this.robotPose = robotPose;
    
    this.camera = new PhotonCamera(cameraName);
    this.estimator = new PhotonPoseEstimator(FIELD_LAYOUT, robotToCamera);
    
    camera.setDriverMode(false);
    camera.setPipelineIndex(APRILTAG_PIPELINE);
}

    @Override
    public void periodic() {
        // This is the only queue reader for this camera.
        var frames = new ArrayList<>(camera.getAllUnreadResults());
        frames.sort(Comparator.comparingDouble(PhotonPipelineResult::getTimestampSeconds));
        
        for (var frame : frames) {
            double timestamp = frame.getTimestampSeconds();
            double age = Timer.getFPGATimestamp() - timestamp;
            
            if (!Double.isFinite(timestamp) || age < -MAX_FUTURE_S || age > MAX_FRAME_AGE_S) {
                reject("Frame timestamp outside limit");
                continue;
            }
            if (timestamp <= lastFrameTime) {
                reject("Duplicate or out-of-order frame");
                continue;
            }
            
            lastFrameTime = timestamp;
            latest = frame; // Empty frames also replace the cache.
            processPose(frame);
        }
        
        field.setRobotPose(robotPose.get());
        SmartDashboard.putBoolean("Vision/Connected", camera.isConnected());
        SmartDashboard.putBoolean("Vision/FreshFrame", isHealthy());
        SmartDashboard.putBoolean("Vision/HasTargets", isHealthy() && latest.hasTargets());
        SmartDashboard.putNumber(
            "Vision/FrameAgeSeconds",
            latest == null ? -1.0 : Timer.getFPGATimestamp() - lastFrameTime
        );
        SmartDashboard.putNumber("Vision/Passed", passedCount);
        SmartDashboard.putNumber("Vision/Rejected", rejectedCount);
        SmartDashboard.putNumber("Vision/Fused", fusedCount);
        SmartDashboard.putString("Vision/LastResult", lastReason);
    }

    private void processPose(PhotonPipelineResult frame) {
        if (!frame.hasTargets()) {
            reject("No targets in frame");
            return;
        }

        var estimate = estimator.estimateCoprocMultiTagPose(frame);
        boolean multi = estimate.isPresent();
        if (estimate.isEmpty()) {
            estimate = estimator.estimateLowestAmbiguityPose(frame);
        }
        if (estimate.isEmpty()) {
            reject("No pose solution");
            return;
        }

        EstimatedRobotPose measurement = estimate.get();
        
        // The 2026 targetsUsed field can include all visible tags,
        // even for a single-tag fallback. Resolve actual contributors.
        List<PhotonTrackedTarget> used;
        if (multi) {
            var ids = frame.getMultiTagResult().get().fiducialIDsUsed;
            used = frame.getTargets().stream()
                .filter(t -> ids.contains((short) t.getFiducialId()))
                .toList();
        } else {
            used = frame.getTargets().stream()
                .filter(t -> Double.isFinite(t.getPoseAmbiguity()) && t.getPoseAmbiguity() >= 0.0)
                .min(Comparator.comparingDouble(PhotonTrackedTarget::getPoseAmbiguity))
                .map(List::of)
                .orElse(List.of());
        }

        String issue = poseIssue(measurement, used, multi);
        if (!issue.isEmpty()) {
            reject(issue);
            return;
        }

        passedCount++;
        Pose2d pose = measurement.estimatedPose.toPose2d();
        field.getObject("LastPassedVision").setPose(pose);

        double meanDistance = used.stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(MAX_MULTI_DISTANCE_M);
            
        double xyStdDev = (multi ? 0.25 : 0.70) - (1.0 + meanDistance * meanDistance / 20.0);

        // A large angular deviation gives the gyro more authority.
        if (ENABLE_POSE_FUSION && CAMERA_TRANSFORM_VERIFIED) {
            consumer.accept(
                pose,
                measurement.timestampSeconds,
                VecBuilder.fill(xyStdDev, xyStdDev, 1.0e6)
            );
            fusedCount++;
            lastReason = "Fused";
        } else {
            lastReason = "Passed; fusion disabled";
        }
    }

    private String poseIssue(
        EstimatedRobotPose measurement,
        List<PhotonTrackedTarget> targets,
        boolean multi
    ) {
        Pose3d pose = measurement.estimatedPose;
        double[] values = {
            pose.getX(), pose.getY(), pose.getZ(),
            pose.getRotation().getX(), pose.getRotation().getY(), pose.getRotation().getZ()
        };
        
        for (double value : values) {
            if (!Double.isFinite(value)) return "Non-finite pose";
        }
        
        if (Math.abs(pose.getZ()) > MAX_Z_ERROR_M) {
            return "Robot height outside limit";
        }
        if (Math.abs(pose.getRotation().getX()) > MAX_TILT_RAD
                || Math.abs(pose.getRotation().getY()) > MAX_TILT_RAD) {
            return "Robot tilt outside limit";
        }
        if (pose.getX() < -FIELD_MARGIN_M
                || pose.getY() < -FIELD_MARGIN_M
                || pose.getX() > FIELD_LAYOUT.getFieldLength() + FIELD_MARGIN_M
                || pose.getY() > FIELD_LAYOUT.getFieldWidth() + FIELD_MARGIN_M) {
            return "Pose outside field";
        }
        if (targets.isEmpty()) return "No contributing targets";
        if (multi && targets.size() < 2) return "Too few MultiTag IDs";
        
        for (var target : targets) {
            if (FIELD_LAYOUT.getTagPose(target.getFiducialId()).isEmpty()) {
                return "Unknown tag ID";
            }
            double distance = target.getBestCameraToTarget().getTranslation().getNorm();
            double limit = multi ? MAX_MULTI_DISTANCE_M : MAX_SINGLE_DISTANCE_M;
            if (!Double.isFinite(distance) || distance <= 0.0 || distance > limit) {
                return "Tag distance limit";
            }
        }
        
        if (!multi && !usableSingle(targets.get(0))) {
            return "Single-tag ambiguity limit";
        }
        
        return "";
    }

    private boolean usableSingle(PhotonTrackedTarget target) {
        double ambiguity = target.getPoseAmbiguity();
        double distance = target.getBestCameraToTarget().getTranslation().getNorm();
        
        return FIELD_LAYOUT.getTagPose(target.getFiducialId()).isPresent()
            && Double.isFinite(ambiguity)
            && ambiguity >= 0.0
            && ambiguity <= MAX_SINGLE_AMBIGUITY
            && Double.isFinite(distance)
            && distance > 0.0
            && distance <= MAX_SINGLE_DISTANCE_M;
    }

    public boolean isHealthy() {
        double age = Timer.getFPGATimestamp() - lastFrameTime;
        return camera.isConnected()
            && latest != null
            && age >= -MAX_FUTURE_S
            && age <= MAX_FRAME_AGE_S;
    }

    public Optional<AimObservation> getAimObservation(int tagId) {
        if (!isHealthy() || !CAMERA_TRANSFORM_VERIFIED) {
            return Optional.empty();
        }
        
        for (var target : latest.getTargets()) {
            if (target.getFiducialId() != tagId || !usableSingle(target)) continue;
            
            var robotToTarget = new Pose3d()
                .transformBy(ROBOT_TO_CAMERA)
                .transformBy(target.getBestCameraToTarget())
                .getTranslation();
                
            double x = robotToTarget.getX();
            double y = robotToTarget.getY();
            
            if (!Double.isFinite(x) || !Double.isFinite(y) || x <= 0.10) continue;
            
            return Optional.of(new AimObservation(Math.atan2(y, x), Math.hypot(x, y)));
        }
        
        return Optional.empty();
    }

    private void reject(String reason) {
        rejectedCount++;
        lastReason = reason;
    }
}
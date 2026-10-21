package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {
    private VisionConstants() {}

    public static final String CAMERA_NAME = "frontleftcam";
    public static final int APRILTAG_PIPELINE = 0;
    
        
    public static final AprilTagFieldLayout FIELD_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // Change these only after the tests in this guide.
    public static final boolean CAMERA_TRANSFORM_VERIFIED = false;
    public static final boolean ENABLE_POSE_FUSION = false;
    public static final boolean ENABLE_AIM_TEST = false;
    public static final int AIM_TAG_ID = -1;

    public static final double MAX_FRAME_AGE_S = 0.25;
    public static final double MAX_FUTURE_S = 0.02;
    public static final double MAX_SINGLE_AMBIGUITY = 0.20;
    public static final double MAX_SINGLE_DISTANCE_M = 4.0;
    public static final double MAX_MULTI_DISTANCE_M = 6.0;
    public static final double MAX_Z_ERROR_M = 0.30;
    public static final double MAX_TILT_RAD = Units.degreesToRadians(15.0);
    public static final double FIELD_MARGIN_M = 0.25;
    public static final double AIM_KP = 1.5;
    public static final double AIM_MAX_RAD_PER_S = 0.5;
    public static final double AIM_TOLERANCE_RAD = Units.degreesToRadians(2.0);

    public static final String FRONTLEFT_CAMERA_NAME = "frontleftcam";
    public static final Transform3d FRONTLEFT_ROBOT_TO_CAMERA =
        new Transform3d(
            new Translation3d(-0.245, 0.261, 0.55),
            new Rotation3d(0.0, Units.degreesToRadians(70), 0.0)
        );

    public static final String FRONTRIGHT_CAMERA_NAME = "frontrightcam";
    public static final Transform3d FRONTRIGHT_ROBOT_TO_CAMERA =
        new Transform3d(
            new Translation3d(0.245, 0.261, 0.55),
            new Rotation3d(0.0, Units.degreesToRadians(70.0), 180.0)
        );
}

// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputs[] inputs;
  private final Alert[] disconnectedAlerts;
  private final StructArrayPublisher<Pose3d>[] tagPosePublishers;
  private final StructArrayPublisher<Pose3d>[] robotPosePublishers;
  private final StructArrayPublisher<Pose3d>[] acceptedPosePublishers;
  private final StructArrayPublisher<Pose3d>[] rejectedPosePublishers;
  private final StructArrayPublisher<Pose3d> allTagPosePublisher;
  private final StructArrayPublisher<Pose3d> allRobotPosePublisher;
  private final StructArrayPublisher<Pose3d> allAcceptedPosePublisher;
  private final StructArrayPublisher<Pose3d> allRejectedPosePublisher;

  @SuppressWarnings("unchecked")
  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputs[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputs();
    }

    // Initialize disconnected alerts and pose logging publishers
    this.disconnectedAlerts = new Alert[io.length];
    this.tagPosePublishers = new StructArrayPublisher[io.length];
    this.robotPosePublishers = new StructArrayPublisher[io.length];
    this.acceptedPosePublishers = new StructArrayPublisher[io.length];
    this.rejectedPosePublishers = new StructArrayPublisher[io.length];
    var table = NetworkTableInstance.getDefault().getTable("Vision");
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert("Vision camera " + io[i].getName() + " is disconnected.", AlertType.kWarning);
      var cameraTable = table.getSubTable(io[i].getName());
      tagPosePublishers[i] =
          cameraTable.getStructArrayTopic("TagPoses", Pose3d.struct).publish();
      robotPosePublishers[i] =
          cameraTable.getStructArrayTopic("RobotPoses", Pose3d.struct).publish();
      acceptedPosePublishers[i] =
          cameraTable.getStructArrayTopic("RobotPosesAccepted", Pose3d.struct).publish();
      rejectedPosePublishers[i] =
          cameraTable.getStructArrayTopic("RobotPosesRejected", Pose3d.struct).publish();
    }

    var summaryTable = table.getSubTable("Summary");
    allTagPosePublisher = summaryTable.getStructArrayTopic("TagPoses", Pose3d.struct).publish();
    allRobotPosePublisher = summaryTable.getStructArrayTopic("RobotPoses", Pose3d.struct).publish();
    allAcceptedPosePublisher =
        summaryTable.getStructArrayTopic("RobotPosesAccepted", Pose3d.struct).publish();
    allRejectedPosePublisher =
        summaryTable.getStructArrayTopic("RobotPosesRejected", Pose3d.struct).publish();
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public boolean hasTarget(int cameraIndex) {
    double targetAge =
        Timer.getFPGATimestamp() - inputs[cameraIndex].targetTimestampSeconds;
    return inputs[cameraIndex].hasTarget
        && inputs[cameraIndex].connected
        && targetAge >= 0.0
        && targetAge < 0.25;
  }

  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    // Update all camera inputs first
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
    }

    // Get cameras with targets and select the best one
    List<Integer> cameras = camerasWithTarget();
    int cameraIndex = cameraOfBestChoice(cameras);

    boolean hasTarget = cameraIndex != -1;

    SmartDashboard.putBoolean(
        "Vision/Aiming/HasTarget",
        hasTarget
    );

    SmartDashboard.putNumber(
        "Vision/Aiming/TxDegrees",
        hasTarget ? getTargetX(cameraIndex).getDegrees() : 0.0
    );

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int i = 0; i < io.length; i++) {
      // Update disconnected alert
      disconnectedAlerts[i].set(!inputs[i].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[i].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[i].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= linearStdDevMegatag2Factor;
        }
        if (i < cameraStdDevFactors.length) {
            linearStdDev *= cameraStdDevFactors[i];
            angularStdDev *= cameraStdDevFactors[i];
        }

        // Send vision observation. The drivetrain converts the FPGA timestamp to the
        // Phoenix current-time base, so the raw observation timestamp is passed through.
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera data
      tagPosePublishers[i].set(tagPoses.toArray(new Pose3d[0]));
      robotPosePublishers[i].set(robotPoses.toArray(new Pose3d[0]));
      acceptedPosePublishers[i].set(robotPosesAccepted.toArray(new Pose3d[0]));
      rejectedPosePublishers[i].set(robotPosesRejected.toArray(new Pose3d[0]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    allTagPosePublisher.set(allTagPoses.toArray(new Pose3d[0]));
    allRobotPosePublisher.set(allRobotPoses.toArray(new Pose3d[0]));
    allAcceptedPosePublisher.set(allRobotPosesAccepted.toArray(new Pose3d[0]));
    allRejectedPosePublisher.set(allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  // Called by AimAtTag to get a list of cameras that have a target
  // io.length is the number of cameras
  public List<Integer> camerasWithTarget() {
    List<Integer> camerasWithTarget = new ArrayList<Integer>();

    // Loop through all the cameras and check if they have a target.
    // If they do, add them to the list.
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      if (hasTarget(cameraIndex)) {
        camerasWithTarget.add(cameraIndex);
      }
    }

    return camerasWithTarget;
  }

  // Called by AimAtTag to get the camera with the best target
  // Usually only helpful if there are multiple cameras with a target
  // but it still selects a camera for AimAtTag to use
  public int cameraOfBestChoice(List<Integer> camerasWithTarget) {
    // If there are no cameras with a target, return -1
    if (camerasWithTarget.isEmpty()) {
      return -1;
    }

    // If there is only one camera with a target, return that camera
    if (camerasWithTarget.size() == 1) {
      return camerasWithTarget.get(0);
    }

    // Decide which camera has the lowest absolute value of tx
    double bestCameraTX = Double.MAX_VALUE;
    int bestCameraIndex = -1;

    // Loop through all the cameras
    for (int cameraIndex = 0; cameraIndex < camerasWithTarget.size(); cameraIndex++) {

      // Get the camera index inside camerasWithTarget
      // This is so if the camera indices start with 1 or above it still works
      int currentCameraIndex = camerasWithTarget.get(cameraIndex);

      // Get the tx of the current camera and compare it to the best camera tx
      double tx = getTargetX(currentCameraIndex).getDegrees();

      // Use absolute value to select the target closest to 0
      if (Math.abs(tx) < Math.abs(bestCameraTX)) {
        bestCameraTX = tx;
        bestCameraIndex = currentCameraIndex;
      }
    }

    return bestCameraIndex;
  }

  // Also called by AimAtTag
  public int getIoLength() {
    return io.length;
  }
}

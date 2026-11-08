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

import frc.robot.commands.AimAtTag;

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
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      
    List<Integer> cameras = camerasWithTarget();
        int cameraIndex = cameraOfBestChoice(cameras);

        SmartDashboard.putBoolean(
            "Vision/Aiming/HasTarget",
            cameraIndex != -1
        );

        if (cameraIndex != -1) {
            double tx = getTargetX(cameraIndex).getDegrees();

            SmartDashboard.putNumber(
                "Vision/Aiming/TxDegrees",
                tx
            );
        } else {
            SmartDashboard.putNumber(
                "Vision/Aiming/TxDegrees",
                0.0
            );
        }
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
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
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation. The drivetrain converts the FPGA timestamp to the
        // Phoenix current-time base, so the raw observation timestamp is passed through.
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera data
      tagPosePublishers[cameraIndex].set(tagPoses.toArray(new Pose3d[0]));
      robotPosePublishers[cameraIndex].set(robotPoses.toArray(new Pose3d[0]));
      acceptedPosePublishers[cameraIndex].set(robotPosesAccepted.toArray(new Pose3d[0]));
      rejectedPosePublishers[cameraIndex].set(robotPosesRejected.toArray(new Pose3d[0]));
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


  //Called by aim at tag to get a list of cameras that have a target
  //Io.length is the number of cameras
  public List<Integer> camerasWithTarget() {


    List<Integer> CamerasWithTarget = new ArrayList<Integer>();

    //loop through all the cameras and check if they have a target, if they do add them to the list
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {

      if (hasTarget(cameraIndex)) {
        CamerasWithTarget.add(cameraIndex);
      }

    }
    return CamerasWithTarget;
  }

  //Called by aim at tag to get the camera with the best target
  //Usually only helpful if there are multiple cameras with a target
  //but it still selects a camera for aim at tag to use
  public int cameraOfBestChoice(List<Integer> CamerasWithTarget) {

    int bestCameraIndex = -1;

    //if there are no cameras with a target, return -1

    if (CamerasWithTarget.isEmpty()) {
      bestCameraIndex = -1;
      return bestCameraIndex;
    }

    //if there is only one camera with a target, return that camera
    if (CamerasWithTarget.size() == 1) {
      bestCameraIndex = CamerasWithTarget.get(0);
      return bestCameraIndex;
    }
    //decide which camera has the lowest value of tx
    int bestCameraTX = Integer.MAX_VALUE;

    //loop through all the cameras
    for (int cameraIndex  = 0; cameraIndex < CamerasWithTarget.size(); cameraIndex++) {

      //get the camera index inside Cameras with target
      //this is so if the camera indices start with
      // 1 or above it still works
      int currentCameraIndex = CamerasWithTarget.get(cameraIndex);

      //we get the tx of the current camera index and compare it to the best camera tx
      //if it's the first iteration then it'll always
      //be smaller since we set bestCameraTX to Integer.MAX_VALUE
      //which is the highest value an integer can be, so itll always be smaller than that
      double tx = getTargetX(currentCameraIndex).getDegrees();

      //if the tx is less than the best camera tx, update the best camera tx and index
      //use abs value to make sure we get the closest to 0, since tx can be negative or positive
      if (Math.abs(tx) < Math.abs(bestCameraTX)) {
        bestCameraTX = (int) tx;
        bestCameraIndex = currentCameraIndex;
      }
    }


    return bestCameraIndex;

  }

  //also called by aim at tag
  public int getIoLength() {
    return io.length;
  }


}

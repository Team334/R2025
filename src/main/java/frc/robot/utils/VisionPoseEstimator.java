// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.FaultLogger;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.Function;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Handles pose estimation coming from a single PhotonVision camera. */
@Logged(strategy = Strategy.OPT_IN)
public class VisionPoseEstimator implements AutoCloseable {
  /** The camera's NT name. */
  @Logged(name = "Camera Name")
  public final String camName;

  /**
   * The ambiguity threshold on this camera to limit incoming vision estimates (used for filtering).
   */
  @Logged(name = "Ambiguity Threshold")
  public final double ambiguityThreshold;

  /**
   * Std devs factor based on this specific camera, increase it if the resolution is lowered on this
   * camera, if the fov is high, if the ambiguity threshold is increased, etc.
   */
  @Logged(name = "Camera Std Devs Factor")
  public final double cameraStdDevsFactor;

  /** The location of the camera relative to the robot's center. */
  @Logged(name = "Robot To Camera Transform")
  public final Transform3d robotToCam;

  /** Maximum allowed distance for single tag estimates. */
  @Logged(name = "Single Tag Max Distance")
  public final double singleTagMaxDistance;

  /** Maximum allowed distance for multitag estimates. */
  @Logged(name = "Multi-Tag Max Distance")
  public final double multiTagMaxDistance;

  @Logged(name = "Camera Mount Angle")
  public final double cameraAngle;

  /**
   * Whether this estimator is ignoring the vision heading estimate (if this is true the vision
   * theta std devs will be super high).
   */
  @Logged(name = "Ignore Theta Estimate")
  public boolean ignoreThetaEstimate = true;

  private final PhotonCamera _camera;
  private final PhotonCameraSim _cameraSim;

  private final PhotonPoseEstimator _poseEstimator;

  private final String _logPath;

  // new estimates from last update call
  private List<VisionPoseEstimate> _newEstimates = new ArrayList<>();

  /** Constants for a single vision pose estimator camera. */
  public record VisionPoseEstimatorConstants(
      /** The NT name of the camera. */
      String camName,

      /** The robot to camera transform */
      Transform3d robotToCam,

      /** The ambiguity threshold for filtering */
      double ambiguityThreshold,

      /** The camera's std devs factor. */
      double cameraStdDevsFactor,

      /** Maximum allowed distance for single tag estimates. */
      double singleTagMaxDistance,

      /** Maximum allowed distance for multitag estimates. */
      double multiTagMaxDistance,

      /** The mounting camera angle. */
      double cameraAngle) {}

  /** Represents a single vision pose estimate. */
  public record VisionPoseEstimate(
      /** The pose to add into the estimator. */
      Pose3d pose,

      /** The timestamp of when the frame was taken. (-1 when no tags). */
      double timestamp,

      /** The ambiguity of this measurement (-1 when no tags or when multi-tag). */
      double ambiguity,

      /** The detected tag ids in this measurement. */
      int[] detectedTags,

      /** The average distance from the tag(s) (-1 when no tags). */
      double avgTagDistance,

      /**
       * The [xMeters, yMeters, thetaRadians] noise standard deviations of this pose estimate ([-1,
       * -1, -1] when no tags or invalid).
       */
      double[] stdDevs,

      /** Whether this estimate passed the filter or not. */
      boolean isValid) {
    /**
     * Used for sorting a list of vision pose estimates, first the timestamps are sorted (from
     * smallest to highest), then the standard deviations at the same timestamp are sorted if
     * necessary.
     */
    public static final Comparator<VisionPoseEstimate> sorter =
        Comparator.comparing(
                VisionPoseEstimate::timestamp,
                (t1, t2) -> {
                  if (t1 > t2) return 1;
                  if (t1 < t2) return -1;
                  return 0;
                })
            .thenComparing(
                // this only happens when measurements land on the same timestamp, they need to be
                // sorted by decreasing std devs
                // this is further explained on the 3rd bullet point here:
                // https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2023-build-thread/420691/36
                VisionPoseEstimate::stdDevs,
                (s1, s2) -> {
                  return -Double.compare(
                      // compare total s1 std devs to total s2 std devs
                      // if s1 (total) is greater than s2 (total), that actually means that we want
                      // s2 to come
                      // after s1 in the sorted array, which is why the negative symbol is needed
                      s1[0] + s1[1] + s1[2], s2[0] + s2[1] + s2[2]);
                });
  }

  /**
   * Builds a new vision pose estimator from a single camera constants. NT instance is set to
   * default, and the field layout is set to whatever is in the constants file.
   */
  public static VisionPoseEstimator buildFromConstants(VisionPoseEstimatorConstants camConstants) {
    return buildFromConstants(
        camConstants, NetworkTableInstance.getDefault(), FieldConstants.fieldLayout);
  }

  /**
   * Builds a new vision pose estimator from a single camera constants. NT instance must be
   * configured, and the field layout must be configured (use this for unit tests).
   */
  public static VisionPoseEstimator buildFromConstants(
      VisionPoseEstimatorConstants camConstants,
      NetworkTableInstance ntInst,
      AprilTagFieldLayout fieldLayout) {
    return new VisionPoseEstimator(
        camConstants.camName,
        camConstants.robotToCam,
        camConstants.singleTagMaxDistance,
        camConstants.multiTagMaxDistance,
        camConstants.ambiguityThreshold,
        camConstants.cameraStdDevsFactor,
        camConstants.cameraAngle,
        ntInst,
        fieldLayout);
  }

  /** Creates a new VisionPoseEstimator (all params are members that are javadocced already). */
  public VisionPoseEstimator(
      String camName,
      Transform3d robotToCam,
      double singleTagMaxDistance,
      double multiTagMaxDistance,
      double ambiguityThreshold,
      double cameraStdDevsFactor,
      double cameraAngle,
      NetworkTableInstance ntInst,
      AprilTagFieldLayout fieldLayout) {
    this.camName = camName;
    this.robotToCam = robotToCam;
    this.ambiguityThreshold = ambiguityThreshold;
    this.cameraStdDevsFactor = cameraStdDevsFactor;
    this.singleTagMaxDistance = singleTagMaxDistance;
    this.multiTagMaxDistance = multiTagMaxDistance;
    this.cameraAngle = cameraAngle;

    _camera = new PhotonCamera(ntInst, camName);

    _poseEstimator =
        new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);

    // this is actually "closest-to-gyro" in the robot code
    _poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    _logPath = "Swerve/" + camName + "/Estimate/";

    FaultLogger.register(_camera);

    if (Robot.isSimulation()) {
      var cameraProps = new SimCameraProperties();

      _cameraSim = new PhotonCameraSim(_camera, cameraProps, fieldLayout);
    } else {
      _cameraSim = null;
    }
  }

  /**
   * Returns an array of the new estimates since the last {@link #update} call. This should be used
   * for the wpilib pose estimator.
   */
  public List<VisionPoseEstimate> getNewEstimates() {
    return _newEstimates;
  }

  // appends a new estimate to the log file
  private void logNewEstimate(VisionPoseEstimate estimate) {
    DogLog.log(_logPath + "Pose", estimate.pose);
    DogLog.log(_logPath + "Timestamp", estimate.timestamp);
    DogLog.log(_logPath + "Ambiguity", estimate.ambiguity);
    DogLog.log(_logPath + "Detected Tags", estimate.detectedTags);
    DogLog.log(_logPath + "Average Tag Distance", estimate.avgTagDistance);
    DogLog.log(_logPath + "Std Devs", estimate.stdDevs);
    DogLog.log(_logPath + "Is Valid", estimate.isValid);
  }

  /**
   * Processes a given {@link EstimatedRobotPose}, converting it into a filtered {@link
   * VisionPoseEstimate} with calculated measurement standard deviations.
   *
   * @param estimate The photon vision estimate.
   * @param gyroHeading The gyro heading at the given estimate timestamp (necessary for
   *     disambiguation).
   * @return A new vision pose estimate.
   */
  private VisionPoseEstimate processEstimate(EstimatedRobotPose estimate, Rotation2d gyroHeading) {
    // estimate properties
    Pose3d estimatedPose = estimate.estimatedPose;
    double timestamp = estimate.timestampSeconds;
    double ambiguity = -1;
    List<PhotonTrackedTarget> targets = estimate.targetsUsed;
    int tagAmount = targets.size();
    int[] detectedTags = new int[tagAmount];
    double avgTagDistance = 0;
    double[] stdDevs = new double[] {-1, -1, -1};
    boolean isValid = false;
    boolean isInAuton = DriverStation.isAutonomous();

    // If there is 1 tag and the Robot is in autonomous then use the trignometric method
    if (tagAmount == 1 && isInAuton) {
      // https://www.chiefdelphi.com/uploads/default/optimized/3X/5/e/5e5c5d9e4297e8e6623a5b438004ed23f4a660c0_2_775x463.jpeg
      // Inspired by
      // https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2025-build-thread/477314/84
      var target = targets.get(0);
      // Get the transform of the target relative to the camera
      // (https://www.chiefdelphi.com/t/interpreting-photonvision-results/419048)
      int tagId = target.getFiducialId();
      Pose3d tagPose = _poseEstimator.getFieldTags().getTagPose(tagId).get();
      // Get Z distance from the robot camera to the tag
      double knownHeightDiff = tagPose.getZ() - robotToCam.getZ();
      // Get the adjacent side of the triangle in https://ibb.co/0jDvcsBK (diagram may be
      // inaccuarate - reviewers may delete this comment)
      double distanceFromCameraToTag = knownHeightDiff / Math.tan(cameraAngle);

      // Transform the tagPose by distanceFromRobotToTag in order to give an estimation of where the
      // robot is
      estimatedPose =
          new Pose3d(
              estimatedPose.getTranslation().getX() - distanceFromCameraToTag,
              estimatedPose.getTranslation().getY(),
              distanceFromCameraToTag,
              estimatedPose.getRotation());
    }
    // ---- DISAMBIGUATE (if single-tag) ----
    // disambiguate poses using gyro measurement (only necessary for a single tag)
    else if (tagAmount == 1) {
      var target = targets.get(0);
      int tagId = target.getFiducialId();
      Pose3d tagPose = _poseEstimator.getFieldTags().getTagPose(tagId).get();

      ambiguity = target.getPoseAmbiguity();

      Pose3d betterReprojPose = tagPose.transformBy(target.getBestCameraToTarget().inverse());
      Pose3d worseReprojPose = tagPose.transformBy(target.getAlternateCameraToTarget().inverse());

      betterReprojPose = betterReprojPose.transformBy(robotToCam.inverse());
      worseReprojPose = worseReprojPose.transformBy(robotToCam.inverse());

      // check which of the poses is closer to the correct gyro heading
      if (Math.abs(betterReprojPose.toPose2d().getRotation().minus(gyroHeading).getDegrees())
          < Math.abs(worseReprojPose.toPose2d().getRotation().minus(gyroHeading).getDegrees())) {
        estimatedPose = betterReprojPose;
      } else {
        estimatedPose = worseReprojPose;
      }
    }

    // ---- FILTER ----
    // get tag distance
    for (int i = 0; i < tagAmount; i++) {
      int tagId = targets.get(i).getFiducialId();
      Pose3d tagPose = _poseEstimator.getFieldTags().getTagPose(tagId).get();
      detectedTags[i] = tagId;
      avgTagDistance += tagPose.getTranslation().getDistance(estimatedPose.getTranslation());
    }

    avgTagDistance /= tagAmount;

    // run all filtering
    boolean badAmbiguity = ambiguity >= ambiguityThreshold;
    boolean outOfBounds =
        (estimatedPose.getX() < -VisionConstants.xBoundMargin
            || estimatedPose.getX()
                > _poseEstimator.getFieldTags().getFieldLength() + VisionConstants.xBoundMargin
            || estimatedPose.getY() < -VisionConstants.yBoundMargin
            || estimatedPose.getY()
                > _poseEstimator.getFieldTags().getFieldWidth() + VisionConstants.yBoundMargin
            || estimatedPose.getZ() < -VisionConstants.zBoundMargin
            || estimatedPose.getZ() > VisionConstants.zBoundMargin);

    boolean tooFar =
        tagAmount == 1
            ? avgTagDistance > singleTagMaxDistance
            : avgTagDistance > multiTagMaxDistance;

    isValid = !(badAmbiguity || outOfBounds || tooFar);

    // ---- STD DEVS CALCULATION ----
    if (isValid) {
      double[] baseStdDevs =
          tagAmount == 1
              ? VisionConstants.singleTagBaseStdDevs
              : VisionConstants.multiTagBaseStdDevs;

      double xStdDevs = baseStdDevs[0] * Math.pow(avgTagDistance, 2) * cameraStdDevsFactor;
      double yStdDevs = baseStdDevs[1] * Math.pow(avgTagDistance, 2) * cameraStdDevsFactor;
      double thetaStdDevs = baseStdDevs[2] * Math.pow(avgTagDistance, 2) * cameraStdDevsFactor;

      if (ignoreThetaEstimate) thetaStdDevs = 999999999;

      stdDevs[0] = xStdDevs;
      stdDevs[1] = yStdDevs;
      stdDevs[2] = thetaStdDevs;
    }

    return new VisionPoseEstimate(
        estimatedPose, timestamp, ambiguity, detectedTags, avgTagDistance, stdDevs, isValid);
  }

  /** Reads from the camera and generates an array of new latest {@link VisionPoseEstimate}(s). */
  public void update(Function<Double, Rotation2d> headingAtTime) {
    _newEstimates.clear(); // reset new estimates

    var results = _camera.getAllUnreadResults();

    DogLog.log(
        "Swerve/" + camName + "/Camera Result #",
        results.size()); // also to check if cam's connected

    for (var result : results) {
      var est = _poseEstimator.update(result);

      if (est.isPresent()) {
        var newEstimate =
            processEstimate(est.get(), headingAtTime.apply(est.get().timestampSeconds));
        _newEstimates.add(newEstimate);

        logNewEstimate(newEstimate);
      }
    }
  }

  /**
   * Returns the camera simulation to add into the vision system simulation. This is null in real
   * life testing.
   */
  public PhotonCameraSim getCameraSim() {
    return _cameraSim;
  }

  @Override
  public void close() {
    _camera.close();
    _cameraSim.close();
  }
}

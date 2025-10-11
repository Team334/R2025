// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.FaultLogger;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.Arrays;
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

  /** The ambiguity threshold on this camera for single-tag estimates. */
  @Logged(name = "Ambiguity Threshold")
  public final double ambiguityThreshold;

  /**
   * Std devs factor based on this specific camera, increase it if the resolution is lowered on this
   * camera, if the fov is high, etc.
   */
  @Logged(name = "Camera Std Devs Factor")
  public final double cameraStdDevsFactor;

  /** The location of the camera relative to the robot's center. */
  @Logged(name = "Robot To Camera Transform")
  public final Transform3d robotToCam;

  /** Maximum allowed distance for single tag estimates. */
  @Logged(name = "Single Tag Max Distance")
  public final double singleTagMaxDistance;

  /** Maximum allowed distance for multi-tag estimates. */
  @Logged(name = "Multi-Tag Max Distance")
  public final double multiTagMaxDistance;

  /**
   * Whether this estimator is ignoring the vision heading estimate (if this is true the vision
   * theta std devs will be super high).
   */
  @Logged(name = "Ignore Theta Estimate")
  public boolean ignoreThetaEstimate = true;

  private final PhotonCamera _camera;
  private final PhotonCameraSim _cameraSim;

  private final PhotonPoseEstimator _poseEstimator;

  // new estimates from last update call
  private final List<VisionPoseEstimate> _newEstimates = new ArrayList<>();

  private final String _estimateLogPath;

  private final Function<Double, Rotation2d> _gyroAtTime;

  /** Constants for a single vision pose estimator camera. */
  public record VisionPoseEstimatorConstants(
      /** The NT name of the camera. */
      String camName,

      /** The robot to camera transform. */
      Transform3d robotToCam,

      /** The ambiguity threshold for filtering. */
      double ambiguityThreshold,

      /** The camera's std devs factor. */
      double cameraStdDevsFactor,

      /** Maximum allowed distance for single tag estimates. */
      double singleTagMaxDistance,

      /** Maximum allowed distance for multi-tag estimates. */
      double multiTagMaxDistance) {}

  public record SingleTagEstimate(
      /** The pose calculated from the trig estimation. */
      Pose3d pose,

      /** The detected tag id. */
      int tag,

      /** The timestamp of when the frame was made. */
      double timestamp,

      /** The distance from this tag. */
      double distance) {}

  /** Represents a single vision pose estimate. */
  public record VisionPoseEstimate(
      /** The disambiguated pose to add into the estimator. */
      Pose3d pose,

      /** The timestamp of when the frame was taken (-1 when no tags). */
      double timestamp,

      /** The ambiguity of this measurement (-1 when no tags or when multi-tag). */
      double ambiguity,

      /** Alternate pose during single-tag measurements that arises from ambiguity. */
      Pose3d altPose,

      /** The detected tag ids in this measurement. */
      int[] detectedTags,

      /** The array of SingleTagEstimtes recived from trig calulcations */
      SingleTagEstimate[] singleTagEstimates,

      /** The average distance from the tag(s) in 3D space (-1 when no tags). */
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
  public static VisionPoseEstimator buildFromConstants(
      VisionPoseEstimatorConstants camConstants, Function<Double, Rotation2d> gyroAtTime) {
    return buildFromConstants(
        camConstants, NetworkTableInstance.getDefault(), FieldConstants.tagLayout, gyroAtTime);
  }

  /**
   * Builds a new vision pose estimator from a single camera constants. NT instance must be
   * configured, and the field layout must be configured (use this for unit tests).
   */
  public static VisionPoseEstimator buildFromConstants(
      VisionPoseEstimatorConstants camConstants,
      NetworkTableInstance ntInst,
      AprilTagFieldLayout fieldLayout,
      Function<Double, Rotation2d> gyroAtTime) {
    return new VisionPoseEstimator(
        camConstants.camName,
        camConstants.robotToCam,
        camConstants.singleTagMaxDistance,
        camConstants.multiTagMaxDistance,
        camConstants.ambiguityThreshold,
        camConstants.cameraStdDevsFactor,
        ntInst,
        fieldLayout,
        gyroAtTime);
  }

  /** Creates a new VisionPoseEstimator. */
  public VisionPoseEstimator(
      String camName,
      Transform3d robotToCam,
      double singleTagMaxDistance,
      double multiTagMaxDistance,
      double ambiguityThreshold,
      double cameraStdDevsFactor,
      NetworkTableInstance ntInst,
      AprilTagFieldLayout fieldLayout,
      Function<Double, Rotation2d> gyroAtTime) {
    this.camName = camName;
    this.robotToCam = robotToCam;
    this.ambiguityThreshold = ambiguityThreshold;
    this.cameraStdDevsFactor = cameraStdDevsFactor;
    this.singleTagMaxDistance = singleTagMaxDistance;
    this.multiTagMaxDistance = multiTagMaxDistance;

    _camera = new PhotonCamera(ntInst, camName);

    _poseEstimator =
        new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);

    // this is actually "closest-to-gyro" in the robot code
    _poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    _estimateLogPath = "Swerve/" + camName + "/Estimate/";

    _gyroAtTime = gyroAtTime;

    FaultLogger.register(_camera);

    if (Robot.isSimulation()) {
      var cameraProps = new SimCameraProperties();

      cameraProps.setCalibError(0.01, 0.001);

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
    DogLog.log(_estimateLogPath + "Pose", estimate.pose);
    DogLog.log(_estimateLogPath + "Timestamp", estimate.timestamp);
    DogLog.log(_estimateLogPath + "Ambiguity", estimate.ambiguity);
    DogLog.log(_estimateLogPath + "Alternate Pose", estimate.altPose);
    DogLog.log(_estimateLogPath + "Detected Tags", estimate.detectedTags);
    DogLog.log(
        _estimateLogPath + "Single Tag Trig Estimates",
        Arrays.stream(estimate.singleTagEstimates).map(e -> e.pose).toArray(Pose3d[]::new));
    DogLog.log(_estimateLogPath + "Average Tag Distance", estimate.avgTagDistance);
    DogLog.log(_estimateLogPath + "Std Devs", estimate.stdDevs);
    DogLog.log(_estimateLogPath + "Is Valid", estimate.isValid);
  }

  /** Gives a single tag estimate using trig. */
  private SingleTagEstimate getSingleTagEstimate(
      PhotonTrackedTarget target, Rotation2d gyroHeading, double timestamp) {
    Translation2d camToTagVector =
        new Translation3d(
                target.getBestCameraToTarget().getTranslation().getNorm(),
                new Rotation3d(
                    0, -Math.toRadians(target.getPitch()), -Math.toRadians(target.getYaw())))
            .rotateBy(robotToCam.getRotation())
            .toTranslation2d()
            .rotateBy(gyroHeading);

    var tagPose = FieldConstants.tagLayout.getTagPose(target.getFiducialId()).get().toPose2d();

    Translation2d fieldToCameraTranslation =
        tagPose.getTranslation().plus(camToTagVector.unaryMinus());

    Translation2d camToRobotTranslation =
        robotToCam.getTranslation().toTranslation2d().unaryMinus().rotateBy(gyroHeading);

    Pose2d robotPose =
        new Pose2d(fieldToCameraTranslation.plus(camToRobotTranslation), gyroHeading);

    return new SingleTagEstimate(
        new Pose3d(robotPose),
        target.getFiducialId(),
        timestamp,
        robotPose.getTranslation().getDistance(tagPose.getTranslation()));
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
    Pose3d altPose = estimatedPose;
    double timestamp = estimate.timestampSeconds;
    double ambiguity = -1;
    int tagAmount = estimate.targetsUsed.size();
    int[] detectedTags = new int[tagAmount];
    SingleTagEstimate[] singleTagEstimates = new SingleTagEstimate[tagAmount];
    double avgTagDistance = 0;
    double[] stdDevs = new double[] {-1, -1, -1};
    boolean isValid = false;

    // ---- DISAMBIGUATE (if single-tag) ----
    // disambiguate poses using gyro measurement (only necessary for a single tag)
    if (tagAmount == 1) {
      var target = estimate.targetsUsed.get(0);
      int tagId = target.getFiducialId();
      Pose3d tagPose = _poseEstimator.getFieldTags().getTagPose(tagId).get();

      ambiguity = target.getPoseAmbiguity();

      Pose3d betterReprojPose = tagPose.transformBy(target.getBestCameraToTarget().inverse());
      Pose3d worseReprojPose = tagPose.transformBy(target.getAlternateCameraToTarget().inverse());

      var robotToCamInverse = robotToCam.inverse();

      betterReprojPose = betterReprojPose.transformBy(robotToCamInverse);
      worseReprojPose = worseReprojPose.transformBy(robotToCamInverse);

      // check which of the poses is closer to the correct gyro heading
      if (Math.abs(betterReprojPose.toPose2d().getRotation().minus(gyroHeading).getDegrees())
          < Math.abs(worseReprojPose.toPose2d().getRotation().minus(gyroHeading).getDegrees())) {
        estimatedPose = betterReprojPose;
        altPose = worseReprojPose;
      } else {
        estimatedPose = worseReprojPose;
        altPose = betterReprojPose;
      }
    }

    // ---- FILTER ----
    // get tag distance
    for (int i = 0; i < tagAmount; i++) {
      PhotonTrackedTarget target = estimate.targetsUsed.get(i);

      int tagId = target.getFiducialId();
      Pose3d tagPose = _poseEstimator.getFieldTags().getTagPose(tagId).get();

      singleTagEstimates[i] = getSingleTagEstimate(target, gyroHeading, timestamp);

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
        estimatedPose,
        timestamp,
        ambiguity,
        altPose,
        detectedTags,
        singleTagEstimates,
        avgTagDistance,
        stdDevs,
        isValid);
  }

  /** Reads from the camera and generates an array of new latest {@link VisionPoseEstimate}(s). */
  public void update() {
    _newEstimates.clear(); // reset new estimates

    var results = _camera.getAllUnreadResults();

    for (var result : results) {
      var est = _poseEstimator.update(result);

      if (est.isPresent()) {
        var newEstimate = processEstimate(est.get(), _gyroAtTime.apply(est.get().timestampSeconds));
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

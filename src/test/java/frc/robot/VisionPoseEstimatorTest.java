package frc.robot;

import static frc.lib.UnitTestingUtil.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.VisionPoseEstimator;
import frc.robot.utils.VisionPoseEstimator.VisionPoseEstimate;
import frc.robot.utils.VisionPoseEstimator.VisionPoseEstimatorConstants;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

public class VisionPoseEstimatorTest {
  private VisionPoseEstimator _testCam;
  private VisionSystemSim _visionSystemSim;

  private static AprilTagFieldLayout _fieldLayout;

  // dummy gyro heading function for disambiguation
  private Rotation2d dummyGyroHeading(double t) {
    return Rotation2d.kZero;
  }

  @BeforeAll
  public static void setupField() {
    List<AprilTag> tags = new ArrayList<>();

    // // add all tags to the field layout
    tags.add(
        new AprilTag(1, new Pose3d(1, 0, 1.2, new Rotation3d(0, 0, -Math.PI)))); // close tag #1
    tags.add(
        new AprilTag(
            2,
            new Pose3d(
                2, 0.5, 0.5, new Rotation3d(0, -0.3, -Math.PI)))); // close tag #2 (for multi-tag)
    tags.add(
        new AprilTag(
            3,
            new Pose3d(
                5, 0.5, 0.5, new Rotation3d(0, -1, -Math.PI)))); // far tag #1 (for distance test)
    tags.add(
        new AprilTag(
            4,
            new Pose3d(1.5, 0, 1, new Rotation3d(0, 0, -Math.PI)))); // close tag #3 (for ambiguity)

    _fieldLayout = new AprilTagFieldLayout(tags, Units.feetToMeters(54), Units.feetToMeters(27));
  }

  @BeforeEach
  public void setup() {
    setupTests();

    var testCam =
        new VisionPoseEstimatorConstants(
            "test-cam",
            new Transform3d(new Translation3d(0, 0, 1), new Rotation3d()),
            0.2,
            0.0001,
            3,
            7);

    _testCam = VisionPoseEstimator.buildFromConstants(testCam, getNtInst(), _fieldLayout);

    _visionSystemSim = new VisionSystemSim("");
    _visionSystemSim.addCamera(_testCam.getCameraSim(), _testCam.robotToCam);
  }

  @AfterEach
  public void close() throws Exception {
    reset(_testCam);
  }

  @Test
  public void noResults() {
    _visionSystemSim.update(Pose2d.kZero);

    _testCam.update(this::dummyGyroHeading);

    // no targets, no new estimates
    assertEquals(0, _testCam.getNewEstimates().size());
  }

  @Test
  public void singleTagResult() {
    _visionSystemSim.addVisionTargets(
        new VisionTargetSim(_fieldLayout.getTagPose(1).get(), TargetModel.kAprilTag36h11, 1));

    _visionSystemSim.update(Pose2d.kZero); // should see the single target

    _testCam.update(this::dummyGyroHeading);

    assertEquals(1, _testCam.getNewEstimates().size());
  }

  @Test
  public void singleTagResults() {
    _visionSystemSim.addVisionTargets(
        new VisionTargetSim(_fieldLayout.getTagPose(1).get(), TargetModel.kAprilTag36h11, 1));

    _visionSystemSim.update(Pose2d.kZero);
    _visionSystemSim.update(Pose2d.kZero); // should see two new results of the single target

    _testCam.update(this::dummyGyroHeading);

    assertEquals(2, _testCam.getNewEstimates().size());
  }

  @Test
  public void singleTagEstimate() {
    _visionSystemSim.addVisionTargets(
        new VisionTargetSim(_fieldLayout.getTagPose(1).get(), TargetModel.kAprilTag36h11, 1));

    _visionSystemSim.update(Pose2d.kZero);

    _testCam.update(this::dummyGyroHeading);

    var estimates = _testCam.getNewEstimates(); // 1 estimate
    assertEquals(1, estimates.size());

    var estimate = estimates.get(0);

    assert estimate.isValid();

    // pose validity
    assertEquals(0, estimate.pose().getX(), 1e-4);
    assertEquals(0, estimate.pose().getY(), 1e-4);
    assertEquals(0, estimate.pose().getZ(), 1e-4);
    assertEquals(0, estimate.pose().getRotation().getX(), 1e-4);
    assertEquals(0, estimate.pose().getRotation().getY(), 1e-4);
    assertEquals(0, estimate.pose().getRotation().getZ(), 1e-4);

    // should see only ID 1
    assertArrayEquals(new int[] {1}, estimate.detectedTags());

    // distance validity
    assertEquals(
        _fieldLayout
            .getTagPose(1)
            .get()
            .getTranslation()
            .getDistance(Pose3d.kZero.getTranslation()),
        estimate.avgTagDistance(),
        1e-4);

    // std devs validity
    assertNotEquals(new int[] {-1, -1, -1}, estimate.stdDevs());
    assertNotEquals(new int[] {0, 0, 0}, estimate.stdDevs());
  }

  // multi-tag does not work due to this issue:
  // https://github.com/PhotonVision/photonvision/issues/1630
  // and the field-relative corner coordinates are used in the multi-tag solvepnp:
  // https://github.com/PhotonVision/photonvision/blob/e8efef476b3b4681c8899a8720774d6dbd5ccf56/photon-targeting/src/main/java/org/photonvision/estimation/OpenCVHelp.java#L573
  // @Test
  public void multiTagEstimate() {
    _visionSystemSim.addAprilTags(_fieldLayout);
    _visionSystemSim.update(Pose2d.kZero);

    _testCam.update(this::dummyGyroHeading);

    var estimates = _testCam.getNewEstimates(); // 1 estimate
    assertEquals(1, estimates.size());

    var estimate = estimates.get(0);

    // System.out.println(estimate);

    assert estimate.isValid();

    // pose validity
    assertEquals(0, estimate.pose().getX(), 1e-4);
    assertEquals(0, estimate.pose().getY(), 1e-4);
    assertEquals(0, estimate.pose().getZ(), 1e-4);
    assertEquals(0, estimate.pose().getRotation().getX(), 1e-4);
    assertEquals(0, estimate.pose().getRotation().getY(), 1e-4);
    assertEquals(0, estimate.pose().getRotation().getZ(), 1e-4);

    // should see only ID 1 and 2
    assertArrayEquals(new int[] {1, 2}, estimate.detectedTags());

    assertEquals(-1, estimate.ambiguity()); // -1 ambiguity, it's not present during multi-tag

    // distance validity
    assertEquals(
        (_fieldLayout
                    .getTagPose(1)
                    .get()
                    .getTranslation()
                    .getDistance(Pose3d.kZero.getTranslation())
                + _fieldLayout
                    .getTagPose(2)
                    .get()
                    .getTranslation()
                    .getDistance(Pose3d.kZero.getTranslation()))
            / 2,
        estimate.avgTagDistance(),
        1e-4);

    // std devs validity
    assertNotEquals(new int[] {-1, -1, -1}, estimate.stdDevs());
    assertNotEquals(new int[] {0, 0, 0}, estimate.stdDevs());
  }

  @Test
  public void boundsFilter() {
    _visionSystemSim.addVisionTargets(
        new VisionTargetSim(_fieldLayout.getTagPose(1).get(), TargetModel.kAprilTag36h11, 1));

    // float the robot above the ground by 0.3 meters
    _visionSystemSim.update(new Pose3d(0, 0, 0.3, Rotation3d.kZero));

    _testCam.update(this::dummyGyroHeading);

    var estimates = _testCam.getNewEstimates(); // 1 estimate
    assertEquals(1, estimates.size());

    var estimate = estimates.get(0);

    assertFalse(estimate.isValid());

    // pose validity
    assertEquals(0, estimate.pose().getX(), 1e-4);
    assertEquals(0, estimate.pose().getY(), 1e-4);
    assertNotEquals(0, estimate.pose().getZ(), 1e-4);
    assertEquals(0, estimate.pose().getRotation().getX(), 1e-4);
    assertEquals(0, estimate.pose().getRotation().getY(), 1e-4);
    assertEquals(0, estimate.pose().getRotation().getZ(), 1e-4);

    // should see only ID 1
    assertArrayEquals(new int[] {1}, estimate.detectedTags());

    // distance validity
    assertNotEquals(
        _fieldLayout
            .getTagPose(1)
            .get()
            .getTranslation()
            .getDistance(Pose3d.kZero.getTranslation()),
        estimate.avgTagDistance(),
        1e-4);

    // std devs validity
    assertArrayEquals(new double[] {-1, -1, -1}, estimate.stdDevs());
  }

  // beta ambiguity issue:
  // https://github.com/PhotonVision/photonvision/issues/1623
  // so this isn't gonna work
  // @Test
  public void ambiguityFilter() {
    // TODO
  }

  @Test
  public void singleTagDistanceFilter() {
    _visionSystemSim.addVisionTargets(
        new VisionTargetSim(_fieldLayout.getTagPose(3).get(), TargetModel.kAprilTag36h11, 3));

    _visionSystemSim.update(Pose2d.kZero);

    _testCam.update(this::dummyGyroHeading);

    var estimates = _testCam.getNewEstimates(); // 1 estimate
    assertEquals(1, estimates.size());

    var estimate = estimates.get(0);

    assertFalse(estimate.isValid());

    // pose validity
    assertEquals(0, estimate.pose().getX(), 1e-4);
    assertEquals(0, estimate.pose().getY(), 1e-4);
    assertEquals(0, estimate.pose().getZ(), 1e-4);
    assertEquals(0, estimate.pose().getRotation().getX(), 1e-4);
    assertEquals(0, estimate.pose().getRotation().getY(), 1e-4);
    assertEquals(0, estimate.pose().getRotation().getZ(), 1e-4);

    // should see only ID 3
    assertArrayEquals(new int[] {3}, estimate.detectedTags());

    // distance validity
    assertEquals(
        _fieldLayout
            .getTagPose(3)
            .get()
            .getTranslation()
            .getDistance(Pose3d.kZero.getTranslation()),
        estimate.avgTagDistance(),
        1e-4);

    // std devs validity
    assertArrayEquals(new double[] {-1, -1, -1}, estimate.stdDevs());
  }

  // multi-tag doesn't work, see other comment about multi-tag above
  // @Test
  public void multiTagDistanceFilter() {
    // TODO
  }

  @Test
  public void estimateSort() {
    List<VisionPoseEstimate> newEstimates = new ArrayList<>();

    for (int i = 3; i > 0; i--) {
      newEstimates.add(
          new VisionPoseEstimate(
              Pose3d.kZero, i, 0.03, new int[] {1}, 1.2, new double[] {0.3, 0.1, 0.2}, true));
    }

    newEstimates.add(
        new VisionPoseEstimate(
            Pose3d.kZero,
            2, // same timestamp case
            0.03,
            new int[] {1},
            1.2,
            new double[] {
              4, 3, 5
            }, // these are worse std devs, so it should come before the better estimate @ timestamp
            // = 2s
            true));

    newEstimates.sort(VisionPoseEstimate.sorter);

    // should be sorted from least timestamp to greatest
    assertEquals(1, newEstimates.get(0).timestamp());
    assertEquals(2, newEstimates.get(1).timestamp());
    assertEquals(2, newEstimates.get(2).timestamp());
    assertEquals(3, newEstimates.get(3).timestamp());

    assertArrayEquals(new double[] {4, 3, 5}, newEstimates.get(1).stdDevs());
    assertArrayEquals(new double[] {0.3, 0.1, 0.2}, newEstimates.get(2).stdDevs());
  }

  // beta ambiguity issue:
  // https://github.com/PhotonVision/photonvision/issues/1623
  // so this isn't gonna work
  // @Test
  public void disambiguation() {
    // TODO
  }
}

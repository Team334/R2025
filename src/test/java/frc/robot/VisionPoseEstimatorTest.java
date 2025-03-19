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
import frc.robot.utils.VisionPoseEstimator.SingleTagEstimate;
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

    // add all tags to the field layout
    tags.add(
        new AprilTag(1, new Pose3d(1, 0, 1.2, new Rotation3d(0, -0.3, -Math.PI)))); // close tag #1
    tags.add(
        new AprilTag(
            2, new Pose3d(2, 0.5, 0.5, new Rotation3d(0, -0.3, -Math.PI)))); // close tag #2
    tags.add(
        new AprilTag(3, new Pose3d(5, 0.5, 0.5, new Rotation3d(0, -1, -Math.PI)))); // far tag #1
    tags.add(
        new AprilTag(4, new Pose3d(7, 0.8, 0.8, new Rotation3d(-0.5, -1, -Math.PI)))); // far tag #2
    tags.add(
        new AprilTag(5, new Pose3d(1.5, 0, 1, new Rotation3d(0, 0, -Math.PI)))); // ambigious tag

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
            5);

    _testCam =
        VisionPoseEstimator.buildFromConstants(
            testCam, getNtInst(), _fieldLayout, this::dummyGyroHeading);

    // specific corner noise for these tests
    _testCam.getCameraSim().prop.setCalibError(0.01, 0.001);

    _visionSystemSim = new VisionSystemSim("");
    _visionSystemSim.addCamera(_testCam.getCameraSim(), _testCam.robotToCam);
  }

  @AfterEach
  public void close() {
    reset(_testCam);
  }

  @Test
  public void noResults() {
    _visionSystemSim.update(Pose2d.kZero);

    _testCam.update();

    // no targets, no new estimates
    assertEquals(0, _testCam.getNewEstimates().size());
  }

  @Test
  public void singleTagResult() {
    _visionSystemSim.addVisionTargets(
        new VisionTargetSim(_fieldLayout.getTagPose(1).get(), TargetModel.kAprilTag36h11, 1));

    _visionSystemSim.update(Pose2d.kZero); // should see the single target

    _testCam.update();

    assertEquals(1, _testCam.getNewEstimates().size());
  }

  @Test
  public void singleTagResults() {
    _visionSystemSim.addVisionTargets(
        new VisionTargetSim(_fieldLayout.getTagPose(1).get(), TargetModel.kAprilTag36h11, 1));

    _visionSystemSim.update(Pose2d.kZero);
    _visionSystemSim.update(Pose2d.kZero); // should see two new results of the single target

    _testCam.update();

    assertEquals(2, _testCam.getNewEstimates().size());
  }

  @Test
  public void singleTagEstimate() {
    _visionSystemSim.addVisionTargets(
        new VisionTargetSim(_fieldLayout.getTagPose(2).get(), TargetModel.kAprilTag36h11, 2));

    _visionSystemSim.update(Pose2d.kZero);

    _testCam.update();

    var estimates = _testCam.getNewEstimates(); // 1 estimate
    assertEquals(1, estimates.size());

    var estimate = estimates.get(0);

    // this test sometimes fails for some unknown reason, so this is here in case
    System.out.println(estimate);

    assert estimate.isValid();

    // pose validity
    assertEquals(0, estimate.pose().getX(), 1e-2);
    assertEquals(0, estimate.pose().getY(), 1e-2);
    assertEquals(0, estimate.pose().getZ(), 1e-2);
    assertEquals(0, estimate.pose().getRotation().getX(), 1e-2);
    assertEquals(0, estimate.pose().getRotation().getY(), 1e-2);
    assertEquals(0, estimate.pose().getRotation().getZ(), 1e-2);

    // should see only ID 2
    assertArrayEquals(new int[] {2}, estimate.detectedTags());

    // distance validity
    assertEquals(
        _fieldLayout
            .getTagPose(2)
            .get()
            .getTranslation()
            .getDistance(Pose3d.kZero.getTranslation()),
        estimate.avgTagDistance(),
        1e-2);

    // std devs validity
    assertNotEquals(new int[] {-1, -1, -1}, estimate.stdDevs());
    assertNotEquals(new int[] {0, 0, 0}, estimate.stdDevs());
  }

  @Test
  public void multiTagEstimate() {
    _visionSystemSim.addAprilTags(_fieldLayout);
    _visionSystemSim.update(Pose2d.kZero);

    _testCam.update();

    var estimates = _testCam.getNewEstimates(); // 1 estimate
    assertEquals(1, estimates.size());

    var estimate = estimates.get(0);

    assert estimate.isValid();

    // pose validity
    assertEquals(0, estimate.pose().getX(), 1e-2);
    assertEquals(0, estimate.pose().getY(), 1e-2);
    assertEquals(0, estimate.pose().getZ(), 1e-2);
    assertEquals(0, estimate.pose().getRotation().getX(), 1e-2);
    assertEquals(0, estimate.pose().getRotation().getY(), 1e-2);
    assertEquals(0, estimate.pose().getRotation().getZ(), 1e-2);

    assert estimate.detectedTags().length == 5;

    assertEquals(-1, estimate.ambiguity()); // -1 ambiguity, it's not present during multi-tag

    // distance validity
    double distance = 0;

    for (AprilTag tag : _fieldLayout.getTags()) {
      distance += tag.pose.getTranslation().getDistance(Pose3d.kZero.getTranslation());
    }

    assertEquals(distance / 5, estimate.avgTagDistance(), 1e-2);

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

    _testCam.update();

    var estimates = _testCam.getNewEstimates(); // 1 estimate
    assertEquals(1, estimates.size());

    var estimate = estimates.get(0);

    assertFalse(estimate.isValid());

    // pose validity
    assertNotEquals(0, estimate.pose().getZ(), 1e-2);

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
        1e-2);

    // std devs validity
    assertArrayEquals(new double[] {-1, -1, -1}, estimate.stdDevs());
  }

  @Test
  public void ambiguityFilter() {
    _visionSystemSim.addVisionTargets(
        new VisionTargetSim(_fieldLayout.getTagPose(5).get(), TargetModel.kAprilTag36h11, 5));

    _visionSystemSim.update(Pose2d.kZero);

    _testCam.update();

    var estimates = _testCam.getNewEstimates(); // 1 estimate
    assertEquals(1, estimates.size());

    var estimate = estimates.get(0);

    assertFalse(estimate.isValid());

    assert estimate.ambiguity() > _testCam.ambiguityThreshold;

    // should see only ID 5
    assertArrayEquals(new int[] {5}, estimate.detectedTags());

    // distance validity (is still accurate in ambigious scenarios)
    assertEquals(
        _fieldLayout
            .getTagPose(5)
            .get()
            .getTranslation()
            .getDistance(Pose3d.kZero.getTranslation()),
        estimate.avgTagDistance(),
        1e-2);

    // std devs validity
    assertArrayEquals(new double[] {-1, -1, -1}, estimate.stdDevs());
  }

  @Test
  public void singleTagDistanceFilter() {
    _visionSystemSim.addVisionTargets(
        new VisionTargetSim(_fieldLayout.getTagPose(3).get(), TargetModel.kAprilTag36h11, 3));

    _visionSystemSim.update(Pose2d.kZero);

    _testCam.update();

    var estimates = _testCam.getNewEstimates(); // 1 estimate
    assertEquals(1, estimates.size());

    var estimate = estimates.get(0);

    assertFalse(estimate.isValid());

    // pose validity
    assertEquals(0, estimate.pose().getX(), 1e-2);
    assertEquals(0, estimate.pose().getY(), 1e-2);
    assertEquals(0, estimate.pose().getZ(), 1e-2);
    assertEquals(0, estimate.pose().getRotation().getX(), 1e-2);
    assertEquals(0, estimate.pose().getRotation().getY(), 1e-2);
    assertEquals(0, estimate.pose().getRotation().getZ(), 1e-2);

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
        1e-2);

    // std devs validity
    assertArrayEquals(new double[] {-1, -1, -1}, estimate.stdDevs());
  }

  @Test
  public void multiTagDistanceFilter() {
    _visionSystemSim.addVisionTargets(
        new VisionTargetSim(_fieldLayout.getTagPose(3).get(), TargetModel.kAprilTag36h11, 3),
        new VisionTargetSim(_fieldLayout.getTagPose(4).get(), TargetModel.kAprilTag36h11, 4));

    _visionSystemSim.update(Pose2d.kZero);

    _testCam.update();

    var estimates = _testCam.getNewEstimates(); // 1 estimate
    assertEquals(1, estimates.size());

    var estimate = estimates.get(0);

    assertFalse(estimate.isValid());

    // pose validity
    assertEquals(0, estimate.pose().getX(), 1e-2);
    assertEquals(0, estimate.pose().getY(), 1e-2);
    assertEquals(0, estimate.pose().getZ(), 1e-2);
    assertEquals(0, estimate.pose().getRotation().getX(), 1e-2);
    assertEquals(0, estimate.pose().getRotation().getY(), 1e-2);
    assertEquals(0, estimate.pose().getRotation().getZ(), 1e-2);

    // should see ID 3 and 4
    assertArrayEquals(new int[] {3, 4}, estimate.detectedTags());

    // should be no ambiguity
    assertEquals(-1, estimate.ambiguity());

    // distance validity
    assertEquals(
        (_fieldLayout
                    .getTagPose(3)
                    .get()
                    .getTranslation()
                    .getDistance(Pose3d.kZero.getTranslation())
                + _fieldLayout
                    .getTagPose(4)
                    .get()
                    .getTranslation()
                    .getDistance(Pose3d.kZero.getTranslation()))
            / 2,
        estimate.avgTagDistance(),
        1e-2);

    // std devs validity
    assertArrayEquals(new double[] {-1, -1, -1}, estimate.stdDevs());
  }

  @Test
  public void estimateSort() {
    List<VisionPoseEstimate> newEstimates = new ArrayList<>();

    for (int i = 3; i > 0; i--) {
      newEstimates.add(
          new VisionPoseEstimate(
              Pose3d.kZero,
              i,
              0.03,
              Pose3d.kZero,
              new int[] {1},
              new SingleTagEstimate[0],
              1.2,
              new double[] {0.3, 0.1, 0.2},
              true));
    }

    newEstimates.add(
        new VisionPoseEstimate(
            Pose3d.kZero,
            2, // same timestamp case
            0.03,
            Pose3d.kZero,
            new int[] {1},
            new SingleTagEstimate[0],
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

  @Test
  public void disambiguation() {
    _visionSystemSim.addVisionTargets(
        new VisionTargetSim(_fieldLayout.getTagPose(2).get(), TargetModel.kAprilTag36h11, 2));

    _visionSystemSim.update(Pose2d.kZero);

    _testCam.update();

    var estimates = _testCam.getNewEstimates(); // 1 estimate
    assertEquals(1, estimates.size());

    var estimate = estimates.get(0);

    // this test sometimes fails for some unknown reason, so this is here in case
    System.out.println(estimate);

    assert estimate.isValid();

    // everything alr checked in singleTagEstimate, so just checking disambiguation here ⤵

    var closerError =
        estimate.pose().getRotation().toRotation2d().minus(dummyGyroHeading(estimate.timestamp()));
    var furtherError =
        estimate
            .altPose()
            .getRotation()
            .toRotation2d()
            .minus(dummyGyroHeading(estimate.timestamp()));

    // pose's heading should be closer to the gyro than altPose's heading
    assert Math.abs(closerError.getRadians()) < Math.abs(furtherError.getRadians());
  }
}

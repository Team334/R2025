// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Poses for alignment. */
public class AlignPoses {
  private final Pose2d _left;
  private final Pose2d _center;
  private final Pose2d _right;

  public static enum AlignSide {
    LEFT,
    CENTER,
    RIGHT
  }

  public Pose2d getLeft() {
    return _left;
  }

  public Pose2d getCenter() {
    return _center;
  }

  public Pose2d getRight() {
    return _right;
  }

  /** Setup align poses for three positions */
  public AlignPoses(Pose2d left, Pose2d center, Pose2d right) {
    _left = left;
    _center = center;
    _right = right;
  }

  /** Setup align pose for center */
  public AlignPoses(Pose2d center) {
    this(center, center, center);
  }

  /** Get pose depending on side. */
  public Pose2d getPose(AlignSide side) {
    switch (side) {
      case LEFT:
        return getLeft();

      case CENTER:
        return getCenter();

      case RIGHT:
        return getRight();

      default:
        return Pose2d.kZero;
    }
  }

  /** Rotates all the poses around a point. */
  public AlignPoses rotateAround(Translation2d point, Rotation2d rot) {
    return new AlignPoses(
        _left.rotateAround(point, rot),
        _center.rotateAround(point, rot),
        _right.rotateAround(point, rot));
  }

  /** Transforms all the poses individually by a translation and rotation. */
  public AlignPoses transform(Translation2d translation, Rotation2d rotation) {
    return new AlignPoses(
        new Pose2d(_left.getTranslation().plus(translation), _left.getRotation().plus(rotation)),
        new Pose2d(
            _center.getTranslation().plus(translation), _center.getRotation().plus(rotation)),
        new Pose2d(_right.getTranslation().plus(translation), _right.getRotation().plus(rotation)));
  }
}

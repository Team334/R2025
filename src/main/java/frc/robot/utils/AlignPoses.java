// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;

/** Poses for alignment. */
public class AlignPoses {
  private final Pose2d _left;
  private final Pose2d _center;
  private final Pose2d _right;

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
}

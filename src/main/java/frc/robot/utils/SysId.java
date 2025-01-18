// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/** Displays SysId routines on SmartDashboard. */
public class SysId {
  /**
   * Displays a single routine to SmartDashboard.
   *
   * @param name The base name of this routine (ex: "Swerve Translation" or "Arm Rotation")
   * @param routine The SysId Routine.
   */
  public static void displayRoutine(String name, SysIdRoutine routine) {
    SmartDashboard.putData(name + " Forward Quasistatic", routine.quasistatic(Direction.kForward));
    SmartDashboard.putData(name + " Reverse Quasistatic", routine.quasistatic(Direction.kReverse));
    SmartDashboard.putData(name + " Forward Dynamic", routine.dynamic(Direction.kForward));
    SmartDashboard.putData(name + " Reverse Dynamic", routine.dynamic(Direction.kReverse));
  }
}

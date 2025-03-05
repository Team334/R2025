// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.BooleanSupplier;

/** Displays SysId routines on SmartDashboard. */
public class SysId {
  public static boolean displayRoutines = false;

  /**
   * Displays a single routine to SmartDashboard.
   *
   * @param name The base name of this routine (ex: "Swerve Translation" or "Arm Rotation").
   * @param routine The SysId Routine.
   */
  public static void displayRoutine(String name, SysIdRoutine routine) {
    displayRoutine(name, routine, () -> false, () -> false);
  }

  /**
   * Displays a single routine to SmartDashboard.
   *
   * @param name The base name of this routine (ex: "Swerve Translation" or "Arm Rotation").
   * @param routine The SysId Routine.
   * @param stopForwardTest A boolean supplier that stops the forward test, could be triggered by a
   *     sensor for example.
   * @param stopReverseTest A boolean supplier that stops the reverse test, could be triggered by a
   *     sensor for example.
   */
  public static void displayRoutine(
      String name,
      SysIdRoutine routine,
      BooleanSupplier stopForwardTest,
      BooleanSupplier stopReverseTest) {
    if (!displayRoutines) return;

    SmartDashboard.putData(
        name + " Forward Quasistatic",
        routine.quasistatic(Direction.kForward).until(stopForwardTest));
    SmartDashboard.putData(
        name + " Reverse Quasistatic",
        routine.quasistatic(Direction.kReverse).until(stopReverseTest));
    SmartDashboard.putData(
        name + " Forward Dynamic", routine.dynamic(Direction.kForward).until(stopForwardTest));
    SmartDashboard.putData(
        name + " Reverse Dynamic", routine.dynamic(Direction.kReverse).until(stopReverseTest));
  }
}

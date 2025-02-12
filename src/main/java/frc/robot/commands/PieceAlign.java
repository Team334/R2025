// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Tuning;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PieceAlign extends Command {
  private final Swerve _swerve;
  private final double _KP = 1;
  private final DoubleEntry _txLog = Tuning.entry("/Tuning/tx", 0.0);

  public PieceAlign(Swerve swerve) {
    _swerve = swerve;

    addRequirements(_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _swerve.drive(0, 0, _txLog.getAsDouble() * _KP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

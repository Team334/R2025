// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

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

  private DoubleSupplier _xInput;
  private DoubleSupplier _yInput;
  private DoubleSupplier _omegaInput;

  private final DoubleEntry _txLog = Tuning.entry("/Tuning/tx", 0.0);

  public PieceAlign(Swerve swerve, DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier omegaInput) {
    _swerve = swerve;

    _xInput = xInput;
    _yInput = yInput;
    _omegaInput = omegaInput;

    addRequirements(_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _swerve.drive(-_xInput.getAsDouble(), -_yInput.getAsDouble(), (_txLog.getAsDouble() * _KP) + -(_omegaInput.getAsDouble()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

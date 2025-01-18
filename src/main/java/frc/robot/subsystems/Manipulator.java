package frc.robot.subsystems;

import static frc.robot.utils.GlobalState.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.AdvancedSubsystem;

public class Manipulator extends AdvancedSubsystem {
  public Manipulator() {
    setDefaultCommand(setSpeed(0));

    new Trigger(() -> getCurrentPiece() == Piece.CORAL).whileTrue(holdCoral());
    new Trigger(() -> getCurrentPiece() == Piece.ALGAE).whileTrue(holdAlgae());
  }

  public static enum Piece {
    CORAL,
    ALGAE,
    NONE
  }

  @Logged(name = "Speed")
  public double getSpeed() {
    return 0;
  }

  /** Set the speed of the manipulator motor in rad/s. */
  public Command setSpeed(double speed) {
    return run(() -> {});
  }

  public Command holdCoral() {
    return run(() -> {});
  }

  public Command holdAlgae() {
    return run(() -> {});
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public void close() throws Exception {}
}

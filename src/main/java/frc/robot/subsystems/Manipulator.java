package frc.robot.subsystems;

import static frc.robot.Robot.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.AdvancedSubsystem;
import java.util.function.Consumer;

public class Manipulator extends AdvancedSubsystem {
  private final Consumer<Piece> _currentPieceSetter; // TODO: this should be used on beam triggers

  public Manipulator(Consumer<Piece> currentPieceSetter) {
    _currentPieceSetter = currentPieceSetter;

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

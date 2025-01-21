package frc.robot.subsystems;

import static frc.robot.Robot.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.AdvancedSubsystem;
import frc.robot.Constants.ManipulatorConstants;
import java.util.function.Consumer;

public class Manipulator extends AdvancedSubsystem {
  private final Consumer<Piece> _currentPieceSetter;

  private final DigitalInput _beam = new DigitalInput(ManipulatorConstants.beamPort);
  private final DigitalInput _limitSwitch = new DigitalInput(ManipulatorConstants.switchPort);

  private final Trigger _beamBroken = new Trigger(this::getBeam);
  private final Trigger _switchPressed = new Trigger(this::getSwitch);

  private final Trigger _beamWithPiece = _beamBroken.and(() -> getCurrentPiece() != Piece.NONE);
  private final Trigger _beamNoPiece = _beamBroken.and(() -> getCurrentPiece() == Piece.NONE);

  public Manipulator(Consumer<Piece> currentPieceSetter) {
    _currentPieceSetter = currentPieceSetter;

    setDefaultCommand(setSpeed(0));

    new Trigger(() -> getCurrentPiece() == Piece.CORAL).whileTrue(holdCoral());
    new Trigger(() -> getCurrentPiece() == Piece.ALGAE).whileTrue(holdAlgae());

    _beamNoPiece.onFalse(
        Commands.runOnce(
            () -> currentPieceSetter.accept(Piece.CORAL))); // coral pick up not from passoff

    _beamWithPiece.onFalse(
        Commands.runOnce(() -> currentPieceSetter.accept(Piece.NONE))); // any piece came out

    _switchPressed.onTrue(
        Commands.runOnce(() -> _currentPieceSetter.accept(Piece.ALGAE))); // algae picked up
  }

  public static enum Piece {
    CORAL,
    ALGAE,
    NONE
  }

  /** A trigger that is true when the beam is broken and current piece is not none. */
  public Trigger getBeamWithPiece() {
    return _beamWithPiece;
  }

  /** A trigger that is true when the beam is broken and current piece is none. */
  public Trigger getBeamNoPiece() {
    return _beamNoPiece;
  }

  @Logged(name = "Speed")
  public double getSpeed() {
    return 0;
  }

  @Logged(name = "Manipulator Beam")
  public boolean getBeam() {
    return !_beam.get();
  }

  @Logged(name = "Manipulator Switch")
  public boolean getSwitch() {
    // TODO: might have two switches
    return _limitSwitch.get();
  }

  /** Set the speed of the back manipulator wheels in rad/s. */
  public Command setSpeed(double speed) {
    return run(() -> {}).withName("Set Speed");
  }

  /** Hold a coral in place. */
  public Command holdCoral() {
    return run(() -> {}).withName("Hold Coral");
  }

  /** Hold an algae in place. */
  public Command holdAlgae() {
    return run(() -> {}).withName("Hold Algae");
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public void close() {}
}

package frc.robot.utils;

import frc.robot.subsystems.Manipulator.Piece;
import frc.robot.subsystems.Wristevator.WristevatorSetpoint;

public class GlobalState {
  private static Piece _currentPiece = Piece.NONE;
  private static WristevatorSetpoint _setpoint = WristevatorSetpoint.HOME;

  /** The current piece in the manipulator. */
  public static Piece getCurrentPiece() {
    return _currentPiece;
  }

  public static void setCurrentPiece(Piece piece) {
    _currentPiece = piece;
  }

  /** The current setpoint of the wristevator. */
  public static WristevatorSetpoint getSetpoint() {
    return _setpoint;
  }

  public static void setSetpoint(WristevatorSetpoint setpoint) {
    _setpoint = setpoint;
  }
}

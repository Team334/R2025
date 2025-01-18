package frc.robot.utils;

import frc.robot.subsystems.Manipulator.Piece;

public class GlobalState {
    private static Piece _currentPiece = Piece.NONE;

    public static Piece getCurrentPiece() {
        return _currentPiece;
    }

    public static void setCurrentPiece(Piece piece) {
        _currentPiece = piece;
    }
}

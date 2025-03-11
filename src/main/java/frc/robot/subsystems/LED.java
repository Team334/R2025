package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Robot.getCurrentPiece;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.Manipulator.Piece;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class LED extends AdvancedSubsystem {
  private final AddressableLED _led;
  private final AddressableLEDBuffer _ledBuffer;

  private Color _allianceColor;
  private Color _algaeColor = new Color("#0acc82");
  private LEDPattern _patternState;

  private final LEDPattern _elevatorHeightMask;

  public LED(DoubleSupplier elevatorHeight, BooleanSupplier homeSwitch) {
    _led = new AddressableLED(LEDConstants.ledPort);
    _ledBuffer = new AddressableLEDBuffer(0);
    _led.setLength(_ledBuffer.getLength());
    _led.start();

    _elevatorHeightMask =
        LEDPattern.progressMaskLayer(
            () -> homeSwitch.getAsBoolean() ? 1 : elevatorHeight.getAsDouble());

    setDefaultCommand(
        run(
            () -> {
              LEDPattern pattern =
                  getCurrentPiece() == Piece.NONE ? LEDPattern.solid(_allianceColor) : scroll();

              runPattern(pattern, true);
            }));
  }

  /** Set the leds to a pattern with the elevator mask overlayed. */
  public Command runPattern(LEDPattern pattern, boolean elevatorMask) {
    return run(
        () -> {
          if (elevatorMask) pattern.mask(_elevatorHeightMask);

          pattern.applyTo(_ledBuffer);
        });
  }

  @Override
  public void periodic() {
    _led.setData(_ledBuffer);

    _allianceColor =
        DriverStation.getAlliance()
            .map(alliance -> alliance == Alliance.Red ? Color.kRed : Color.kBlue)
            .orElse(Color.kLavender);
  }

  public LEDPattern scroll() {
    LEDPattern pattern =
        getCurrentPiece() == Piece.CORAL
            ? LEDPattern.steps(
                Map.of(
                    0, _allianceColor, 0.25, Color.kWhite, 0.5, _allianceColor, 0.75, Color.kWhite))
            : LEDPattern.steps(
                Map.of(
                    0, _allianceColor, 0.25, _algaeColor, 0.5, _allianceColor, 0.75, _algaeColor));

    return pattern.scrollAtRelativeSpeed(Percent.per(Second).of(50));
  }

  public void stateLogic() {
    // Piece currPiece = Robot.getCurrentPiece();

    // if (_swerve.aligningTo()) {
    //     LEDPattern base = LEDPattern.solid(Color.kOrange);
    //     _patternState = base.blink(Seconds.of(0.15));
    // } else if (_swerve.aligningToPose()) {
    //     _patternState = LEDPattern.solid(Color.kOrange);
    // } else if (currPiece == Piece.ALGAE) {
    //     LEDPattern base = LEDPattern.steps(Map.of(0, _allianceColor, 0.25, _algaeColor, 0.5,
    // _allianceColor, 0.75, _algaeColor));
    //     _patternState = base.scrollAtRelativeSpeed(Percent.per(Second).of(50));
    // } else if (currPiece == Piece.CORAL) {
    //     LEDPattern base = LEDPattern.steps(Map.of(0, _allianceColor, 0.25, Color.kWhite, 0.5,
    // _allianceColor, 0.75, Color.kWhite));
    //     _patternState = base.scrollAtRelativeSpeed(Percent.per(Second).of(50));
    // } else {
    //     _patternState = LEDPattern.solid(_allianceColor);
    // }

    // if (_wristevator.getHeight() > 0) {
    //     LEDPattern mask = LEDPattern.progressMaskLayer(() -> _wristevator.getHeight() /
    // Constants.WristevatorConstants.maxElevatorHeight.in(Radians));
    //     _patternState = _patternState.mask(mask);
    // }

    // _patternState.applyTo(_ledBuffer);
  }

  // private Command runPattern(LEDPattern pattern) {
  //     return run(() -> stateLogic());
  // }
}

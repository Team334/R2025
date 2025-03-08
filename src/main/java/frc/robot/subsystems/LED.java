package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;
import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Manipulator.Piece;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.util.Color;

public class LED extends SubsystemBase {
    private final Wristevator _wristevator;

    private final AddressableLED _led;
    private final AddressableLEDBuffer _ledBuffer;

    private Color _allianceColor;
    private LEDPattern _patternState;

    public LED(int pwmPort, int ledCount, Wristevator wristevator) {
        _wristevator = wristevator;

        _led = new AddressableLED(pwmPort);
        _ledBuffer = new AddressableLEDBuffer(ledCount);
        _led.setLength(_ledBuffer.getLength());
        _led.start();

        Optional<Alliance> alliance = DriverStation.getAlliance();
        _allianceColor = alliance.map(a -> a == Alliance.Red ? Color.kRed : Color.kBlue).orElse(Color.kLavender);

        setDefaultCommand(runPattern(_patternState));
    }

    @Override
    public void periodic() {
        _led.setData(_ledBuffer);
    }

    public void stateLogic() {
        Piece currPiece = Robot.getCurrentPiece();

        if (currPiece == Piece.ALGAE) {
            LEDPattern base = LEDPattern.steps(Map.of(0, _allianceColor, 0.25, Color.kCyan, 0.5, _allianceColor, 0.75, Color.kCyan));
            _patternState = base.scrollAtRelativeSpeed(Percent.per(Second).of(50));
        } else if (currPiece == Piece.CORAL) {
            LEDPattern base = LEDPattern.steps(Map.of(0, _allianceColor, 0.25, Color.kWhite, 0.5, _allianceColor, 0.75, Color.kWhite));
            _patternState = base.scrollAtRelativeSpeed(Percent.per(Second).of(50));
        } else {
            _patternState = LEDPattern.solid(_allianceColor);
        }

        _patternState.applyTo(_ledBuffer);
    }

    public void elevator() {
        LEDPattern mask = LEDPattern.progressMaskLayer(() -> _wristevator.getHeight() / Constants.WristevatorConstants.maxElevatorHeight.in(Radians));
        _patternState = _patternState.mask(mask);
        _patternState.applyTo(_ledBuffer);
    }

    public void elevatorSet() {
        
    }

    public Command runPattern(LEDPattern pattern) {
        return run(() -> stateLogic());
    }

    public Command runElevatorLED() {
        return run(() -> elevator());
    }
}

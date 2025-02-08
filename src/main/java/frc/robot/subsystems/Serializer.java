package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.lib.Tuning;
import frc.robot.Constants.SerializerConstants;
import frc.robot.Robot;
import frc.robot.subsystems.Manipulator.Piece;
import java.util.function.Consumer;

public class Serializer extends AdvancedSubsystem {
  private final DigitalInput _frontBeam;
  private final DigitalInput _backBeam;

  private DIOSim _frontBeamSim;
  private DIOSim _backBeamSim;

  private BooleanEntry _frontBeamSimValue;
  private BooleanEntry _backBeamSimValue;

  private final TalonFX _feedMotor = new TalonFX(SerializerConstants.feedMotorId);

  private final VelocityVoltage _feedVelocitySetter = new VelocityVoltage(0);
  private final StatusSignal<AngularVelocity> _feedVelocityGetter = _feedMotor.getVelocity();

  private final Consumer<Piece> _currentPieceSetter;

  public Serializer(Consumer<Piece> currentPieceSetter) {
    setDefaultCommand(idle());

    _currentPieceSetter = currentPieceSetter;

    _frontBeam = new DigitalInput(SerializerConstants.frontBeamPort);
    _backBeam = new DigitalInput(SerializerConstants.backBeamPort);

    if (Robot.isSimulation()) {
      _frontBeamSim = new DIOSim(_frontBeam);
      _backBeamSim = new DIOSim(_backBeam);

      _frontBeamSimValue = Tuning.entry("/Tuning/Serializer Front Beam", false);
      _backBeamSimValue = Tuning.entry("/Tuning/Serializer Back Beam", false);
    }

    var feedMotorConfigs = new TalonFXConfiguration();

    CTREUtil.attempt(() -> _feedMotor.getConfigurator().apply(feedMotorConfigs), _feedMotor);

    FaultLogger.register(_feedMotor);
  }

  @Logged(name = "Speed")
  public double getSpeed() {
    return _feedVelocityGetter.refresh().getValue().in(RadiansPerSecond);
  }

  // Set the speed of the front feed wheels in rad/s.
  private Command setSpeed(double speed) {
    return run(
        () -> {
          _feedMotor.setControl(_feedVelocitySetter.withVelocity(Units.radiansToRotations(speed)));
        });
  }

  /**
   * Whether a coral is at all inside the serializer.
   *
   * @return front beam broken OR back beam broken
   */
  public boolean hasCoral() {
    return getFrontBeam() || getBackBeam();
  }

  @Logged(name = "Front Beam")
  public boolean getFrontBeam() {
    return !_frontBeam.get();
  }

  @Logged(name = "Back Beam")
  public boolean getBackBeam() {
    return !_backBeam.get();
  }

  public Command idle() {
    return setSpeed(0).withName("Idle");
  }

  /** Intakes a coral until the front beam is broken. */
  public Command intake() {
    return setSpeed(0).until(this::getFrontBeam).withName("Intake");
  }

  /** Outtakes a coral to the intake. */
  public Command outtake() {
    return setSpeed(0).withName("Outtake");
  }

  /** Passoffs a coral to the manipulator. */
  public Command passoff() {
    return setSpeed(0).withName("Passoff");
  }

  /** Inverse passoff from the manipulator. */
  public Command inversePassoff() {
    return setSpeed(0)
        .until(this::getBackBeam)
        .andThen(Commands.runOnce(() -> _currentPieceSetter.accept(Piece.NONE)))
        .withName("Inverse Passoff");
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    _frontBeamSim.setValue(!_frontBeamSimValue.get());
    _backBeamSim.setValue(!_backBeamSimValue.get());
  }

  @Override
  public void close() {
    _frontBeam.close();
    _backBeam.close();

    _feedMotor.close();

    _frontBeamSimValue.close();
    _backBeamSimValue.close();
  }
}

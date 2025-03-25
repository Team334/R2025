// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.*;
import static frc.robot.Constants.WristevatorConstants.Preset.*;

import choreo.auto.AutoChooser;
import com.ctre.phoenix6.SignalLogger;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.FaultLogger;
import frc.lib.InputStream;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.WristevatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Superstructure;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Manipulator.Piece;
import frc.robot.subsystems.Serializer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wristevator;
import frc.robot.utils.AlignPoses;
import frc.robot.utils.AlignPoses.AlignSide;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
@Logged(strategy = Strategy.OPT_IN)
public class Robot extends TimedRobot {
  // controllers
  private final CommandXboxController _driverController =
      new CommandXboxController(Ports.driverController);

  private final CommandXboxController _operatorController =
      new CommandXboxController(Ports.operatorController);

  @Logged(name = "Swerve")
  private final Swerve _swerve = TunerConstants.createDrivetrain();

  @Logged(name = "Intake")
  private final Intake _intake = new Intake();

  @Logged(name = "Serializer")
  private final Serializer _serializer = new Serializer((Piece piece) -> _currentPiece = piece);

  @Logged(name = "Manipulator")
  private final Manipulator _manipulator = new Manipulator((Piece piece) -> _currentPiece = piece);

  @Logged(name = "Wristevator")
  private final Wristevator _wristevator = new Wristevator();

  private final Autos _autos =
      new Autos(
          _swerve,
          (Piece piece) -> _currentPiece = piece,
          _wristevator,
          _manipulator,
          _intake,
          _serializer);
  private final AutoChooser _autoChooser = new AutoChooser();

  private final NetworkTableInstance _ntInst;

  private boolean _fileOnlySet = false;

  // global state variables
  private static Piece _currentPiece = Piece.NONE;

  /** The current piece in the manipulator. */
  public static Piece getCurrentPiece() {
    return _currentPiece;
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    this(NetworkTableInstance.getDefault());
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot(NetworkTableInstance ntInst) {
    _ntInst = ntInst;

    // set up loggers
    DogLog.setOptions(new DogLogOptions().withCaptureDs(true));

    setFileOnly(false); // file-only once connected to fms

    Epilogue.bind(this);
    SignalLogger.start();

    DriverStation.silenceJoystickConnectionWarning(isSimulation());

    FaultLogger.setup(_ntInst);

    configureDefaultCommands();
    configureDriverBindings();
    configureOperatorBindings();

    new Trigger(() -> getCurrentPiece() == Piece.NONE).onChange(rumbleControllers(1, 1));

    SmartDashboard.putData(
        "Robot Self Check",
        sequence(
                runOnce(() -> DataLogManager.log("Robot Self Check Started!")),
                _swerve.fullSelfCheck(),
                runOnce(() -> DataLogManager.log("Robot Self Check Successful!")))
            .withName("Robot Self Check"));

    SmartDashboard.putData(new WheelRadiusCharacterization(_swerve));
    SmartDashboard.putData(
        runOnce(FaultLogger::clear).ignoringDisable(true).withName("Clear Faults"));

    // set up auto chooser
    _autoChooser.addRoutine("Ground 3 Piece", _autos::ground3P);
    _autoChooser.addRoutine("One Piece", _autos::onePiece);

    SmartDashboard.putData("Auto Chooser", _autoChooser);

    autonomous().whileTrue(_autoChooser.selectedCommandScheduler());

    addPeriodic(FaultLogger::update, 1);
  }

  // set logging to be file only or not
  private void setFileOnly(boolean fileOnly) {
    DogLog.setOptions(DogLog.getOptions().withNtPublish(!fileOnly));

    if (fileOnly) {
      Epilogue.getConfig().backend = new FileBackend(DataLogManager.getLog());
      return;
    }

    // if doing both file and nt logging, use the datalogger multilogger setup
    Epilogue.getConfig().backend =
        EpilogueBackend.multi(
            new NTEpilogueBackend(_ntInst), new FileBackend(DataLogManager.getLog()));
  }

  private void configureDefaultCommands() {
    _swerve.setDefaultCommand(
        _swerve.drive(
            InputStream.of(_driverController::getLeftY)
                .negate()
                .signedPow(2)
                .scale(SwerveConstants.maxTranslationalSpeed.in(MetersPerSecond)),
            InputStream.of(_driverController::getLeftX)
                .negate()
                .signedPow(2)
                .scale(SwerveConstants.maxTranslationalSpeed.in(MetersPerSecond)),
            InputStream.of(_driverController::getRightX)
                .negate()
                .signedPow(2)
                .scale(SwerveConstants.maxAngularSpeed.in(RadiansPerSecond))));

    new Trigger(_wristevator::isManual)
        .onTrue(
            _wristevator
                .setSpeeds(
                    InputStream.of(_operatorController::getRightY)
                        .deadband(0.1, 1)
                        .negate()
                        .scale(WristevatorConstants.manualElevatorSpeed.in(RadiansPerSecond)),
                    InputStream.of(_operatorController::getLeftY)
                        .deadband(0.07, 1)
                        .negate()
                        .scale(WristevatorConstants.manualWristSpeed.in(RadiansPerSecond)))
                .ignoringDisable(true));
  }

  private void alignmentTriggers(Trigger button, AlignPoses poses, boolean startReversed) {
    button
        .and(_driverController.leftTrigger().and(_driverController.rightTrigger().negate()))
        .whileTrue(_swerve.alignTo(poses, AlignSide.LEFT, startReversed));

    button
        .and(
            _driverController.leftTrigger().negate().and(_driverController.rightTrigger().negate()))
        .whileTrue(_swerve.alignTo(poses, AlignSide.CENTER, startReversed));

    button
        .and(_driverController.rightTrigger().and(_driverController.leftTrigger().negate()))
        .whileTrue(_swerve.alignTo(poses, AlignSide.RIGHT, startReversed));
  }

  private void configureDriverBindings() {
    _driverController.a().whileTrue(_swerve.brake());
    _driverController.povUp().onTrue(_swerve.toggleFieldOriented());
    _driverController.povDown().onTrue(_swerve.resetHeading());

    // align to piece
    _driverController.leftBumper().whileTrue(_swerve.alignToPiece());

    _driverController.povRight().onTrue(_swerve.resetToReefTag().andThen(rumbleControllers(1, 1)));

    alignmentTriggers(_driverController.x(), FieldConstants.reef, false);
    alignmentTriggers(_driverController.y(), FieldConstants.human, true);
    alignmentTriggers(_driverController.b(), FieldConstants.processor, false);
    alignmentTriggers(_driverController.start(), FieldConstants.cage, false);
  }

  private void configureOperatorBindings() {
    // wristevator setpoint control
    _operatorController.back().onTrue(_wristevator.setGoal(PROCESSOR));
    _operatorController.start().onTrue(_wristevator.setGoal(HUMAN));
    _operatorController.rightStick().onTrue(_wristevator.setGoal(HOME));

    _operatorController.a().onTrue(_wristevator.setGoal(L1));

    _operatorController
        .b()
        .onTrue(
            either(
                _wristevator.setGoal(L2),
                _wristevator.setGoal(LOWER_ALGAE),
                () -> getCurrentPiece() == Piece.CORAL));

    _operatorController
        .y()
        .onTrue(
            either(
                _wristevator.setGoal(L3),
                _wristevator.setGoal(UPPER_ALGAE),
                () -> getCurrentPiece() == Piece.CORAL));

    _operatorController.x().onTrue(_wristevator.setGoal(L4));

    _operatorController.povDown().onTrue(_wristevator.switchToManual());

    // ground intake / passoff
    _operatorController
        .rightBumper()
        .and(_wristevator::homeSwitch)
        .whileTrue(Superstructure.passoff(_intake, _serializer, _manipulator));

    _operatorController
        .rightBumper()
        .and(() -> !_wristevator.homeSwitch())
        .whileTrue(
            Superstructure.groundIntake(_intake, _serializer)
                .andThen(new ScheduleCommand(rumbleControllers(1, 1))));

    // switch to fast manipulator feed mode
    _operatorController
        .leftBumper()
        .onTrue(runOnce(() -> _manipulator.setFastFeed(true)))
        .onFalse(runOnce(() -> _manipulator.setFastFeed(false)));

    _operatorController.povUp().whileTrue(_intake.outtake());

    // intake / inverse passoff
    _operatorController
        .rightTrigger()
        .and(_wristevator::homeSwitch)
        .whileTrue(Superstructure.inversePassoff(_serializer, _manipulator));

    _operatorController
        .rightTrigger()
        .and(() -> !_wristevator.homeSwitch())
        .whileTrue(_manipulator.intake());

    // outtake
    _operatorController.leftTrigger().whileTrue(_manipulator.outtake());
  }

  /** Rumble the driver and operator controllers for some amount of seconds. */
  private Command rumbleControllers(double rumble, double seconds) {
    return run(() -> {
          _driverController.getHID().setRumble(RumbleType.kBothRumble, rumble);
          _operatorController.getHID().setRumble(RumbleType.kBothRumble, rumble);
        })
        .finallyDo(
            () -> {
              _driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
              _operatorController.getHID().setRumble(RumbleType.kBothRumble, 0);
            })
        .withTimeout(seconds);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    if (DriverStation.isFMSAttached() && !_fileOnlySet) {
      setFileOnly(true);

      _fileOnlySet = true;
    }

    DogLog.log("Manipulator Current Piece", _currentPiece);
  }

  @Override
  public void autonomousPeriodic() {
    _autos.ground3P().poll();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void close() {
    super.close();

    _swerve.close();
    _wristevator.close();
    _manipulator.close();
    _intake.close();
    _serializer.close();
  }
}

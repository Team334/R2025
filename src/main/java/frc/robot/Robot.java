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
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.ClassPreloader;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.FaultLogger;
import frc.lib.InputStream;
import frc.robot.Constants.Piece;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.WristevatorConstants.Setpoint;
import frc.robot.commands.Autos;
import frc.robot.commands.Superstructure;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Serializer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wristevator;
import java.lang.reflect.Field;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
@Logged(strategy = Strategy.OPT_IN)
public class Robot extends TimedRobot {
  private final CommandXboxController _driverController =
      new CommandXboxController(Ports.driverController);

  private final CommandXboxController _operatorController =
      new CommandXboxController(Ports.operatorController);

  @Logged(name = "Swerve")
  private final Swerve _swerve = TunerConstants.createDrivetrain();

  @Logged(name = "Intake")
  private final Intake _intake = new Intake();

  @Logged(name = "Serializer")
  private final Serializer _serializer = new Serializer();

  @Logged(name = "Manipulator")
  private final Manipulator _manipulator =
      new Manipulator((Piece piece) -> _manipulatorPiece = piece);

  @Logged(name = "Wristevator")
  private final Wristevator _wristevator =
      new Wristevator((Setpoint goal) -> _wristevatorGoal = goal);

  private final Autos _autos = new Autos(_swerve, _intake);

  private final NetworkTableInstance _ntInst;

  private boolean _fileOnlySet = false;

  private static Piece _manipulatorPiece = Piece.NONE;

  /** The current piece in the manipulator. */
  public static Piece getManipulatorPiece() {
    return _manipulatorPiece;
  }

  private static Setpoint _wristevatorGoal = HOME;

  /** The goal for the wristevator. */
  public static Setpoint getWristevatorGoal() {
    return _wristevatorGoal;
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
    DogLog.setOptions(DogLog.getOptions().withCaptureDs(true));
    // DogLog.setPdh(new PowerDistribution());

    setFileOnly(false); // file-only once connected to fms

    Epilogue.bind(this);
    SignalLogger.start(); // TODO: log canivore can data as well

    DriverStation.silenceJoystickConnectionWarning(isSimulation());

    FaultLogger.setup(_ntInst);

    configureDriverBindings();
    configureOperatorBindings();

    new Trigger(() -> getManipulatorPiece() == Piece.NONE)
        .onChange(rumbleControllers(1, 1).onlyIf(teleop()));

    SmartDashboard.putData(
        "Robot Self Check",
        sequence(
                runOnce(() -> DataLogManager.log("Robot Self Check Started")),
                _swerve.fullSelfCheck(),
                runOnce(() -> DataLogManager.log("Robot Self Check Finished")))
            .withName("Robot Self Check"));

    SmartDashboard.putData(
        runOnce(FaultLogger::clear).withName("Clear Faults").ignoringDisable(true));

    addPeriodic(FaultLogger::update, 1);

    AutoChooser chooser = new AutoChooser();

    chooser.addRoutine("Short Path", _autos::shortPath);
    chooser.addRoutine("Forward Intake Right", _autos::forwardIntakeRight);

    SmartDashboard.putData("Auto Chooser", chooser);

    autonomous().whileTrue(chooser.selectedCommandScheduler());

    choreoSetup();
  }

  /** Watchdog config / class preloading needed to reduce choreo delay. */
  private void choreoSetup() {
    // something slow about watchdog's printEpochs() when there's a loop overrun (Tracer
    // printEpochs() DS writes?)
    // more here: https://www.chiefdelphi.com/t/choreo-autonomous-loop-overruns/495597/21
    // problem now is that loop overruns won't get noticed so need to find another way to log them
    final double loopOverrunWarningPeriod = 30;

    try {
      Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      watchdogField.setAccessible(true);
      Watchdog watchdog = (Watchdog) watchdogField.get(this);
      watchdog.setTimeout(loopOverrunWarningPeriod);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to increase watchdog timeout", false);
    }

    CommandScheduler.getInstance().setPeriod(loopOverrunWarningPeriod);

    // preloading long-loading classes used on auton init by choreo
    ClassPreloader.preload(
        "edu.wpi.first.math.geometry.Transform2d",
        "edu.wpi.first.math.geometry.Twist2d",
        "java.lang.FdLibm$Hypot",
        "choreo.trajectory.Trajectory",
        "choreo.trajectory.SwerveSample");
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

  private void configureDriverBindings() {
    _swerve.setDefaultCommand(
        _swerve.drive(
            InputStream.of(_driverController::getLeftY)
                .negate()
                .scale(SwerveConstants.maxTranslationalSpeed.in(MetersPerSecond)),
            InputStream.of(_driverController::getLeftX)
                .negate()
                .scale(SwerveConstants.maxTranslationalSpeed.in(MetersPerSecond)),
            InputStream.of(_driverController::getRightX)
                .negate()
                .scale(SwerveConstants.maxAngularSpeed.in(RadiansPerSecond))));

    _driverController.x().whileTrue(_swerve.brake());
    _driverController.a().onTrue(_swerve.toggleFieldOriented());
    _driverController.y().onTrue(_swerve.resetHeading());
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
                () -> getManipulatorPiece() == Piece.CORAL));

    _operatorController
        .y()
        .onTrue(
            either(
                _wristevator.setGoal(L3),
                _wristevator.setGoal(UPPER_ALGAE),
                () -> getManipulatorPiece() == Piece.CORAL));

    _operatorController.x().onTrue(_wristevator.setGoal(L4));

    // SmartDashboard.putData(_wristevator.setGoal(PROCESSOR).withName("Processor"));
    // SmartDashboard.putData(_wristevator.setGoal(HUMAN).withName("Human"));
    // SmartDashboard.putData(_wristevator.setGoal(HOME).withName("Home"));
    // SmartDashboard.putData(_wristevator.setGoal(L1).withName("L1"));
    // SmartDashboard.putData(_wristevator.setGoal(L2).withName("L2"));
    // SmartDashboard.putData(_wristevator.setGoal(LOWER_ALGAE).withName("Lower Algae"));
    // SmartDashboard.putData(_wristevator.setGoal(L3).withName("L3"));
    // SmartDashboard.putData(_wristevator.setGoal(UPPER_ALGAE).withName("Upper Algae"));
    // SmartDashboard.putData(_wristevator.setGoal(L4).withName("L4"));

    // ground outtake
    _operatorController.leftBumper().whileTrue(_intake.outtake());
    _operatorController.povUp().whileTrue(Superstructure.serializerOuttake(_serializer, _intake));

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

    // intake / inverse passoff
    _operatorController
        .rightTrigger()
        .and(_wristevator::homeSwitch)
        .whileTrue(Superstructure.inversePassoff(_serializer, _manipulator));

    _operatorController
        .rightTrigger()
        .and(() -> !_wristevator.homeSwitch())
        .whileTrue(_manipulator.feed());
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

    DogLog.log("Manipulator Current Piece", getManipulatorPiece());
    DogLog.log("Wristevator Goal", getWristevatorGoal().toString());

    if (DriverStation.isFMSAttached() && !_fileOnlySet) {
      setFileOnly(true);

      _fileOnlySet = true;
    }
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
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.omegabytes.library.OmegaLib.ControllerTypeBeat.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.Autos.*;
// import frc.robot.commands.Songs.*;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Position;
import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  // private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final Swerve s_Swerve = new Swerve();

  private Position driverTargetPosition = Position.CHASSIS;

  // Commands
  // private final DriveCommand driveCommand = new DriveCommand(driveSubsystem);
  // private final ArmCommand armCommand = new ArmCommand(armSubsystem);
  // private final IntakeCommand intakeCommand = new
  // IntakeCommand(intakeSubsystem);

  private final PlaceCommand placeCommand = new PlaceCommand(intakeSubsystem);
  private final IdleCommand idleCommand = new IdleCommand(intakeSubsystem);

  // Auto Chooser
  private final SendableChooser<Command> chooser = new SendableChooser<>();

  private final SendableChooser<ArmSubsystem.Position> positionChooser = new SendableChooser<>();

  // Songs
  // private final ItsBeenSoLong itsBeenSoLong = new
  // ItsBeenSoLong(driveSubsystem);
  // private final GiornosTheme giornosTheme = new GiornosTheme(driveSubsystem);
  // private final SwedenC418 swedenC418 = new SwedenC418(driveSubsystem);
  // private final MichaelHunterThemeFromSanAndreas
  // michaelHunterThemeFromSanAndreas = new
  // MichaelHunterThemeFromSanAndreas(driveSubsystem);
  // private final Megalovania megalovania = new Megalovania(driveSubsystem);
  // private final bohemianRhapsody bohemianRhapsody = new
  // bohemianRhapsody(driveSubsystem);

  // Drive Controls
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // Driver Buttons
  // private final JoystickButton robotCentric = new
  // JoystickButton(Constants.dXboxController,
  // XboxController.Button.kLeftBumper.value);

  // Manip buttons
  private final JoystickButton zeroGyro = new JoystickButton(Constants.mXboxController,
      XboxController.Button.kBack.value);

  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -Constants.dXboxController.getRawAxis(translationAxis),
            () -> -Constants.dXboxController.getRawAxis(strafeAxis),
            () -> -Constants.dXboxController.getRawAxis(rotationAxis),
            () -> false // robotCentric.getAsBoolean()
        ));
    intakeSubsystem.setDefaultCommand(idleCommand);
    configureBindings();

    // Auto Routines
    chooser.addOption("No auto", null);
    chooser.setDefaultOption("Mobility", new StraightLineAuto1(s_Swerve));
    chooser.addOption("Charge Station", new ChargeStationRedSideAuto(s_Swerve));

    // Songs
    // chooser.addOption("It's Been So Long by The Living Tombstone",
    // itsBeenSoLong);
    // chooser.addOption("Ginornos Theme", giornosTheme);
    // chooser.addOption("Sweden by C418", swedenC418);
    // chooser.addOption("Michael Hunter Theme From San Andreas",
    // michaelHunterThemeFromSanAndreas);
    // chooser.addOption("Megalovania", megalovania);
    // chooser.addOption("Bohemian Rhapsody by Queen", bohemianRhapsody);

    SmartDashboard.putData("Autonomous routine", chooser);

    if (Constants.armPositionDebugChooser) {
      for (Position pos : Position.values()) {
        positionChooser.addOption(pos.toString(), pos);
      }
      SmartDashboard.putData("Position chooser", positionChooser);
    }
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  /*
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController Xbox}/{@link
   * edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link
   * edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    /* DRIVER BINDS */

    // Driver arm controls
    new JoystickButton(dXboxController, XboxController.Button.kA.value)
      .onTrue(Commands.runOnce(() -> driverTargetPosition = Position.GRID_LOW));
    new JoystickButton(dXboxController, XboxController.Button.kB.value)
      .onTrue(Commands.runOnce(() -> driverTargetPosition = Position.GRID_MID));
    new JoystickButton(dXboxController, XboxController.Button.kY.value)
      .onTrue(Commands.runOnce(() -> driverTargetPosition = Position.GRID_HIGH));
    // new JoystickButton(Constants.dXboxController, XboxController.Button.kX.value)
    //   .onTrue(Commands.runOnce(() -> driverTargetPosition = Position.CHASSIS));

    Trigger armTrigger = 
      new JoystickButton(dXboxController, XboxController.Button.kRightBumper.value)
        .whileTrue(Commands.runOnce(() -> armSubsystem.setTargetPosition(armPositionDebugChooser ? positionChooser.getSelected() : driverTargetPosition))
          .andThen(new ArmCommand(armSubsystem)));
    if (!armPositionDebugDirect) {
      armTrigger
        .onFalse(Commands.runOnce(() -> armSubsystem.setTargetPosition(Position.CHASSIS))
          .andThen(new ArmCommand(armSubsystem)));
    }


    // TODO Move the TriggerButton, and make the intakeCommand actually work (runs
    // forever, stops when command terminates, nice to have if finished when piece
    // acquired)
    // new JoystickButton(Constants.dXboxController,
    // XboxController.Button.kLeftBumper.value)
    // .whileTrue(
    // Commands.runOnce(() ->
    // armSubsystem.setTargetPosition(Position.INTAKE_SUBSTATION))
    // .andThen(new ArmCommand(armSubsystem)))
    // .onFalse(
    // Commands.runOnce(() -> Commands.runOnce(() ->
    // armSubsystem.setTargetPosition(Position.CHASSIS)))
    // .andThen(new ArmCommand(armSubsystem)));

    // new JoystickButton(Constants.dXboxController,
    // XboxController.Button.kX.value).onTrue(Commands.runOnce(() ->
    // intakeSubsystem.toggleCube()));

    new JoystickButton(Constants.dXboxController, XboxController.Button.kX.value).onTrue(Commands.runOnce(() -> intakeSubsystem.toggleCube()));

    Trigger driverLeftTrigger = new Trigger(
        () -> Constants.dXboxController.getLeftTriggerAxis() > Constants.triggerAxisThreshold);
    Trigger driverRightTrigger = new Trigger(
        () -> Constants.dXboxController.getRightTriggerAxis() > Constants.triggerAxisThreshold);

    // TODO Move to ground intake position as well
    driverLeftTrigger.whileTrue(new InstantCommand(() -> new IntakeCommand(intakeSubsystem)))
      .onFalse(new InstantCommand(() -> System.out.println("Driver left trigger released")));
    
    driverRightTrigger.onTrue(new InstantCommand(() -> new PlaceCommand(intakeSubsystem)))
      .onFalse(new InstantCommand(() -> System.out.println("Driver right trigger released")));

    /* Manip Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
  }
}
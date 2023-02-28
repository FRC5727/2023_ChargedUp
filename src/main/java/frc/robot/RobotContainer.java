// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.Autos.*;
import frc.robot.commands.Autos.RED.ChargeStationRedSideAuto;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Position;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
//  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final Swerve s_Swerve = new Swerve();

  private Position driverTargetPosition = Position.CHASSIS;

  // Commands
  // private final DriveCommand driveCommand = new DriveCommand(driveSubsystem);
  private final ArmCommand armCommand = new ArmCommand(armSubsystem);
  private final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem);
  private final PlaceCommand placeCommand = new PlaceCommand(intakeSubsystem);
  private final IdleCommand idleCommand = new IdleCommand(intakeSubsystem);
  SendableChooser<Command> chooser = new SendableChooser<>();
  // Auto Routines 
  // private final Auto auto = new Auto(s_Swerve);
  // private final StraightLineAuto1 straightLineAuto1 = new StraightLineAuto1(s_Swerve);
  // private final StraightLineRedToCargo4Auto straightLineRedToCargo4Auto = new StraightLineRedToCargo4Auto(s_Swerve);
  private final ChargeStationRedSideAuto chargeStationRedSideAuto = new ChargeStationRedSideAuto(s_Swerve);
  private final RED2CubeAutoLeft red2CubeAutoLeft = new RED2CubeAutoLeft(s_Swerve);
  private final ChargeStationRedMobility chargeStationRedMobility = new ChargeStationRedMobility(s_Swerve);

  // Drive Controls
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // Driver Buttons
  // private final JoystickButton robotCentric = new JoystickButton(Constants.dXboxController, XboxController.Button.kLeftBumper.value);

  // Manip buttons 
  private final JoystickButton zeroGyro = new JoystickButton(Constants.mXboxController, XboxController.Button.kBack.value);
  
	public RobotContainer() {
    s_Swerve.setDefaultCommand(
      new TeleopSwerve(
        s_Swerve,
        () -> -Constants.dXboxController.getRawAxis(translationAxis),
        () -> -Constants.dXboxController.getRawAxis(strafeAxis),
        () -> -Constants.dXboxController.getRawAxis(rotationAxis),
        () -> false // robotCentric.getAsBoolean()
      )
    );
    intakeSubsystem.setDefaultCommand(idleCommand);
    configureBindings();

    //Auto Routines
    chooser.setDefaultOption("Do Nothin", null);
    chooser.addOption("RED SIDE: Charge Station Auto", chargeStationRedSideAuto);
    chooser.addOption("RED SIDE: Charge Station + Mobility", chargeStationRedMobility);
    chooser.addOption("RED SIDE: 1 Cube Mobility", red2CubeAutoLeft);
    
   
    SmartDashboard.putData(chooser);

  }
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
  /* 
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    /* DRIVER BINDS */
    // Driver arm controls
    // TODO Re-enable arm controls
    // new JoystickButton(Constants.dXboxController, XboxController.Button.kA.value).onTrue(Commands.runOnce(() -> driverTargetPosition = Position.GRID_LOW));
    // new JoystickButton(Constants.dXboxController, XboxController.Button.kB.value).onTrue(Commands.runOnce(() -> driverTargetPosition = Position.GRID_MID));
    // new JoystickButton(Constants.dXboxController, XboxController.Button.kY.value).onTrue(Commands.runOnce(() -> driverTargetPosition = Position.GRID_HIGH));
    // // new JoystickButton(Constants.dXboxController, XboxController.Button.kX.value).onTrue(Commands.runOnce(() -> driverTargetPosition = Position.CHASSIS));
    // new JoystickButton(Constants.dXboxController, XboxController.Button.kRightBumper.value)
    //   .whileTrue(
    //     Commands.runOnce(() -> armSubsystem.setTargetPosition(driverTargetPosition))
    //     .andThen(new ArmCommand(armSubsystem)))
    //   .onFalse(
    //     Commands.runOnce(() -> armSubsystem.setTargetPosition(Position.CHASSIS))
    //     .andThen(new ArmCommand(armSubsystem)));

    // new JoystickButton(Constants.dXboxController, XboxController.Button.kLeftBumper.value)
    // .whileTrue(
    //   Commands.runOnce(() -> armSubsystem.setTargetPosition(Position.INTAKE_SUBSTATION))
    //   .andThen(new ArmCommand(armSubsystem)))
    // .onFalse(
    //   Commands.runOnce(() -> Commands.runOnce(() -> armSubsystem.setTargetPosition(Position.CHASSIS)))
    //   .andThen(new ArmCommand(armSubsystem)));

    new JoystickButton(Constants.dXboxController, XboxController.Button.kX.value).onTrue(Commands.runOnce(() -> intakeSubsystem.toggleCube()));

    Trigger driverLeftTrigger = new Trigger(
      () -> Constants.dXboxController.getLeftTriggerAxis() > 0.05);
    Trigger driverRightTrigger = new Trigger(
      () -> Constants.dXboxController.getRightTriggerAxis() > 0.05);

    driverLeftTrigger.whileTrue(new InstantCommand(() -> new IntakeCommand(intakeSubsystem)))
    .onFalse(new InstantCommand(() -> System.out.println("Driver left trigger released")));

    driverRightTrigger.onTrue(new InstantCommand(() -> new PlaceCommand(intakeSubsystem)))
      .onFalse(new InstantCommand(() -> System.out.println("Driver right trigger released")));
      if(Constants.dXboxController.getLeftTriggerAxis() > 0.05)
    
    /* Manip Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
  }
}
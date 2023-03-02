// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.Autos.*;
import frc.robot.commands.Autos.RED.ChargeStationRedSideAuto;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Position;
import static frc.robot.Constants.*;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

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

  // Auto Chooser
  private final SendableChooser<Command> chooser = new SendableChooser<>();

  private final SendableChooser<ArmSubsystem.Position> positionChooser = new SendableChooser<>();
  // Auto Routines 
  private final ChargeStationRedSideAuto chargeStationRedSideAuto = new ChargeStationRedSideAuto(s_Swerve);
  private final RED2CubeAutoLeft red2CubeAutoLeft = new RED2CubeAutoLeft(s_Swerve);
  private final ChargeStationRedMobility chargeStationRedMobility = new ChargeStationRedMobility(s_Swerve);

  // Drive Controls
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // Driver Buttons
  // private final JoystickButton robotCentric = new
  // JoystickButton(Constants.dXboxController,
  // XboxController.Button.kLeftBumper.value);

  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -Constants.dXboxController.getRawAxis(translationAxis),
            () -> -Constants.dXboxController.getRawAxis(strafeAxis),
            () -> -Constants.dXboxController.getRawAxis(rotationAxis),
            () -> false, // robotCentric.getAsBoolean()
            () -> s_Swerve.getSpeedLimitXY(),
            () -> s_Swerve.getSpeedLimitRot()
        ));
    intakeSubsystem.setDefaultCommand(new IdleCommand(intakeSubsystem));
    configureBindings();

    // Auto Routines
    chooser.setDefaultOption("RED SIDE: Charge Station + Mobility", chargeStationRedMobility);
    chooser.addOption("RED SIDE: 2 Cube Auto Left (Untested)", red2CubeAutoLeft);
    chooser.addOption("No auto", null);
    chooser.addOption("RED SIDE: Charge Station Auto", chargeStationRedSideAuto);

    String pathName = "Blue Simple Test";
    double autoMaxVel = 0.5;
    double autoMaxAccel = 0.3;
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(autoMaxVel, autoMaxAccel));
    HashMap<String, Command> eventMap = new HashMap<>();

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      (() -> s_Swerve.getPose()), // Pose2d supplier
      ((Pose2d pose) -> s_Swerve.resetOdometry(pose)), // Pose2d consumer, used to reset odometry at the beginning of auto
      Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
      new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      ((SwerveModuleState[] desiredStates) -> s_Swerve.setModuleStates(desiredStates)), // Module states consumer used to output to the drive subsystem
      eventMap,
      true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      s_Swerve // The drive subsystem. Used to properly set the requirements of path following commands
    );

    Command simpleAuto = autoBuilder.fullAuto(pathGroup);
    chooser.addOption(pathName, simpleAuto);

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

    Trigger driverLeftBumper = new JoystickButton(Constants.dXboxController, XboxController.Button.kLeftBumper.value);
    Trigger driverRightBumper = new JoystickButton(Constants.dXboxController, XboxController.Button.kRightBumper.value);
    Trigger driverLeftTrigger = new Trigger(
        () -> Constants.dXboxController.getLeftTriggerAxis() > Constants.triggerAxisThreshold);
    Trigger driverRightTrigger = new Trigger(
        () -> Constants.dXboxController.getRightTriggerAxis() > Constants.triggerAxisThreshold);
  
    // Move to selected position
    Trigger armTrigger = 
      driverRightBumper.whileTrue(
        Commands.runOnce(() -> s_Swerve.enableSpeedLimit())
          .andThen(Commands.runOnce(() -> armSubsystem.setTargetPosition(armPositionDebugChooser ? positionChooser.getSelected() : driverTargetPosition)))
          .andThen(new ArmCommand(armSubsystem)));
    if (!armPositionDebugDirect) {
      armTrigger
        .onFalse(
          Commands.waitSeconds(0.5)
            .andThen(Commands.runOnce(() -> s_Swerve.disableSpeedLimit()))
          .alongWith(
            Commands.runOnce(() -> armSubsystem.setTargetPosition(Position.CHASSIS))
              .andThen(new ArmCommand(armSubsystem))));
    }

    Trigger intakeSubstationTrigger = 
      driverRightTrigger.whileTrue(new IntakeCommand(intakeSubsystem)
        .alongWith(
            Commands.runOnce(() -> s_Swerve.enableSpeedLimit())
              .andThen(Commands.runOnce(() -> armSubsystem.setTargetPosition(Position.INTAKE_SUBSTATION)))
              .andThen(new ArmCommand(armSubsystem))));
 
    if (!armPositionDebugDirect) {
      intakeSubstationTrigger
        .onFalse(
          Commands.waitSeconds(0.5)
            .andThen(Commands.runOnce(() -> s_Swerve.disableSpeedLimit()))
          .alongWith(
            Commands.runOnce(() -> armSubsystem.setTargetPosition(Position.CHASSIS))
              .andThen(new ArmCommand(armSubsystem))));
    }
        
    Trigger intakeGroundTrigger = 
      driverLeftTrigger.whileTrue(new IntakeCommand(intakeSubsystem)
        .alongWith(
          Commands.runOnce(() -> armSubsystem.setTargetPosition(Position.INTAKE_GROUND))
            .andThen(new ArmCommand(armSubsystem))));
    
    if (!armPositionDebugDirect) {
      intakeGroundTrigger
        .onFalse(
          Commands.runOnce(() -> armSubsystem.setTargetPosition(Position.CHASSIS))
          .andThen(new ArmCommand(armSubsystem)));
    }

    // Place currently held game piece
    // TODO If we can align automatically, add this to arm commands
    driverLeftBumper.whileTrue(new PlaceCommand(intakeSubsystem));

    // Toggle between cones and cubes
    new JoystickButton(Constants.dXboxController, XboxController.Button.kX.value).onTrue(Commands.runOnce(() -> intakeSubsystem.toggleCube()));

    /* Manip Buttons */
    SmartDashboard.putData("Zero Gyro", Commands.runOnce(() -> s_Swerve.zeroGyro()));
  }
}
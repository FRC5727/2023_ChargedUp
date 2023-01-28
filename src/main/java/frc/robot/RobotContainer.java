// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Auto;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final DriveCommand driveCommand = new DriveCommand(driveSubsystem);
  SendableChooser<Command> chooser = new SendableChooser<>();
  public final Auto auto = new Auto(driveSubsystem);
	public RobotContainer() {
    driveSubsystem.setDefaultCommand(driveCommand);
    configureBindings();
    chooser.setDefaultOption("Auto", auto);
    chooser.addOption("New 2 Ball Left", auto);
    chooser.addOption("New 2 Ball Right", auto);
    chooser.addOption("New 1 Ball", auto);
    chooser.addOption("New 3 Ball", auto);
    SmartDashboard.putData(chooser);
    //getAutonomousCommand();
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
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    
  //}
  public void updateAngle() {
    driveSubsystem.updateAngle();
  }
}


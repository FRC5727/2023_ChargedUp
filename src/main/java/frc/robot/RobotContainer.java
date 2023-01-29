// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.Autos.Auto;
import frc.robot.commands.Songs.GiornosTheme;
import frc.robot.commands.Songs.ItsBeenSoLong;
import frc.robot.commands.Songs.Megalovania;
import frc.robot.commands.Songs.MichaelHunterThemeFromSanAndreas;
import frc.robot.commands.Songs.SwedenC418;
import frc.robot.subsystems.DriveSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //Subsystems
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  //Commands
  private final DriveCommand driveCommand = new DriveCommand(driveSubsystem);
  
  SendableChooser<Command> chooser = new SendableChooser<>();
  //Auto Routines 
  private final Auto auto = new Auto(driveSubsystem);

  //Songs
  private final ItsBeenSoLong itsBeenSoLong = new ItsBeenSoLong(driveSubsystem);
  private final GiornosTheme giornosTheme = new GiornosTheme(driveSubsystem);
  private final SwedenC418 swedenC418 = new SwedenC418(driveSubsystem);
  private final MichaelHunterThemeFromSanAndreas michaelHunterThemeFromSanAndreas = new MichaelHunterThemeFromSanAndreas(driveSubsystem);
  private final Megalovania megalovania = new Megalovania(driveSubsystem);
	public RobotContainer() {
    driveSubsystem.setDefaultCommand(driveCommand);
    configureBindings();
    //Auto Routines
    chooser.setDefaultOption("Auto 1", auto);
    chooser.addOption("Auto 2", auto);
    chooser.addOption("Auto 3", auto);
    chooser.addOption("Auto 4", auto);
    //Songs
    chooser.addOption("It's Been So Long by The Living Tombstone", itsBeenSoLong);
    chooser.addOption("Ginornos Theme", giornosTheme);
    chooser.addOption("Sweden by C418", swedenC418);
    chooser.addOption("Michael Hunter Theme From San Andreas", michaelHunterThemeFromSanAndreas);
    chooser.addOption("Megalovania", megalovania);
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


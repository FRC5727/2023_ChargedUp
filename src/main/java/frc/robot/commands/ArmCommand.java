// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HighArmSubsystem;
import frc.robot.subsystems.LowArmSubsystem;


public class ArmCommand extends CommandBase {
  /** Creates a new ArmCommand. */
  
  private final ArmSubsystem arm;

  public ArmCommand(ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    addRequirements(arm);
  }

  // public void chassisPos(){
  //   arm.setLowArmGoal(0);
  //   arm.setHighArmGoal(0);
  // }
  // public void intakeGroundPos(){
  //   arm.setLowArmGoal(36);
  //   arm.setHighArmGoal(-29);
  // }
  // public void highPos(){
  //   arm.setLowArmGoal(54);
  //   arm.setHighArmGoal(186);
  // }
  // public void midPos(){
  //   arm.setLowArmGoal(26);
  //   arm.setHighArmGoal(121);
  // }
  // public void lowPos(){
  //   arm.setLowArmGoal(27);
  //   arm.setHighArmGoal(2);
  // }
  // public void stationPickupPos(){
  //   arm.setLowArmGoal(0);
  //   arm.setHighArmGoal(0);
  // }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (Constants.dXboxController.getBButtonPressed()) arm.midPos();
    // if (Constants.dXboxController.getAButtonPressed()) arm.lowPos();
    // // if (Constants.dXboxController.getXButtonReleased()) chassisPos();
    // if (Constants.dXboxController.getXButtonPressed()) arm.highPos();
    // if (Constants.dXboxController.getYButtonPressed()) arm.intakeGroundPos();
    // if (Constants.dXboxController.getRightBumperReleased()) stationPickupPos();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

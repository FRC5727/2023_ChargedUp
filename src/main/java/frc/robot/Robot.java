// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  //private DriveSubsystem driveSubsystem;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    DriverStation.silenceJoystickConnectionWarning(true);
    ctreConfigs = new CTREConfigs();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    if (!DriverStation.isFMSAttached())
      PathPlannerServer.startServer(5811);
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
    //CANCoders Absolute Position Values
    // SmartDashboard.putNumber("Front Left Encoder Absolute Position Value: ", fle.getAbsolutePosition());
    // SmartDashboard.putNumber("Front Right Encoder Absolute Position Value: ", fre.getAbsolutePosition());
    // SmartDashboard.putNumber("Rear Right Encoder Absolute Position Value: ", rre.getAbsolutePosition());
    // SmartDashboard.putNumber("Rear Left Encoder Absolute Position Value: ", rle.getAbsolutePosition());
    // SmartDashboard.putNumber("lower coder", lowerArm.getAbsolutePosition());
    // SmartDashboard.putNumber("high coder", highArm.getAbsolutePosition());
    // SmartDashboard.putNumber("Front Left Encoder  Position Value: ", fle.getPosition());
    // SmartDashboard.putNumber("Front Right Encoder  Position Value: ", fre.getPosition());
    // SmartDashboard.putNumber("Rear Right Encoder  Position Value: ", rre.getPosition());
    // SmartDashboard.putNumber("Rear Left Encoder  Position Value: ", rle.getPosition());
    // //CANCoders Bus Voltage
    // SmartDashboard.putNumber("Front Left Encoder Bus Voltage: ", fle.getBusVoltage());
    // SmartDashboard.putNumber("Front Right Encoder Bus Voltage: ", fre.getBusVoltage());
    // SmartDashboard.putNumber("Rear Right Encoder Bus Voltage: ", rre.getBusVoltage());
    // SmartDashboard.putNumber("Rear Left Encoder Bus Voltage: ", rle.getBusVoltage());
    // //CANCoders Velocity 
    // SmartDashboard.putNumber("Front Left Encoder Velocity: ", fle.getVelocity());
    // SmartDashboard.putNumber("Front Right Encoder Velocity: ", fre.getVelocity());
    // SmartDashboard.putNumber("Rear Right Encoder Velocity: ", rre.getVelocity());
    // SmartDashboard.putNumber("Rear Left Encoder Velocity: ", rle.getVelocity());
    // //Falcons / Talon FX: Speed in percent
    // SmartDashboard.putNumber("Front Left Drive Motor Speed: ", FLDMTalon.getMotorOutputPercent());
    // SmartDashboard.putNumber("Front Left Steer Motor Speed: ", FLSMTalon.getMotorOutputPercent());
    // SmartDashboard.putNumber("Front Right Drive Motor Speed: ", FRDMTalon.getMotorOutputPercent());
    // SmartDashboard.putNumber("Front Right Steer Motor Speed: ", FRSMTalon.getMotorOutputPercent());
    // SmartDashboard.putNumber("Rear Right Drive Motor Speed: ", RRDMTalon.getMotorOutputPercent());
    // SmartDashboard.putNumber("Rear Right Steer Motor Speed: ", RRSMTalon.getMotorOutputPercent());
    // SmartDashboard.putNumber("Rear Left Drive Motor Speed: ", RLDMTalon.getMotorOutputPercent());
    // SmartDashboard.putNumber("Rear Left Steer Motor Speed: ", RLSMTalon.getMotorOutputPercent());
    // //Falcons / Talon FX: Temps
    // SmartDashboard.putNumber("Front Left Drive Motor Temp: ", FLDMTalon.getTemperature());
    // SmartDashboard.putNumber("Front Left Steer Motor Temp: ", FLSMTalon.getTemperature());
    // SmartDashboard.putNumber("Front Right Drive Motor Temp: ", FRDMTalon.getTemperature());
    // SmartDashboard.putNumber("Front Right Steer Motor Temp: ", FRSMTalon.getTemperature());
    // SmartDashboard.putNumber("Rear Right Drive Motor Temp: ", RRDMTalon.getTemperature());
    // SmartDashboard.putNumber("Rear Right Steer Motor Temp: ", RRSMTalon.getTemperature());
    // SmartDashboard.putNumber("Rear Left Drive Motor Temp: ", RLDMTalon.getTemperature());
    // SmartDashboard.putNumber("Rear Left Steer Motor Temp: ", RLSMTalon.getTemperature());
    // //Falcons / Talon FX: Bus Voltage
    // SmartDashboard.putNumber("Front Left Drive Motor Bus Voltage: ", FLDMTalon.getBusVoltage());
    // SmartDashboard.putNumber("Front Left Steer Motor Bus Voltage: ", FLSMTalon.getBusVoltage());
    // SmartDashboard.putNumber("Front Right Drive Motor Bus Voltage: ", FRDMTalon.getBusVoltage());
    // SmartDashboard.putNumber("Front Right Steer Motor Bus Voltage: ", FRSMTalon.getBusVoltage());
    // SmartDashboard.putNumber("Rear Right Drive Motor Bus Voltage: ", RRDMTalon.getBusVoltage());
    // SmartDashboard.putNumber("Rear Right Steer Motor Bus Voltage: ", RRSMTalon.getBusVoltage());
    // SmartDashboard.putNumber("Rear Left Drive Motor Bus Voltage: ", RLDMTalon.getBusVoltage());
    // SmartDashboard.putNumber("Rear Left Steer Motor Bus Voltage: ", RLSMTalon.getBusVoltage());
    // //Falcons / Talon FX: Output Voltage
    // SmartDashboard.putNumber("Front Left Drive Motor Output Voltage: ", FLDMTalon.getMotorOutputVoltage());
    // SmartDashboard.putNumber("Front Left Steer Motor Output Voltage: ", FLSMTalon.getMotorOutputVoltage());
    // SmartDashboard.putNumber("Front Right Drive Motor Output Voltage: ", FRDMTalon.getMotorOutputVoltage());
    // SmartDashboard.putNumber("Front Right Steer Motor Output Voltage: ", FRSMTalon.getMotorOutputVoltage());
    // SmartDashboard.putNumber("Rear Right Drive Motor Output Voltage: ", RRDMTalon.getMotorOutputVoltage());
    // SmartDashboard.putNumber("Rear Right Steer Motor Output Voltage: ", RRSMTalon.getMotorOutputVoltage());
    // SmartDashboard.putNumber("Rear Left Drive Motor Output Voltage: ", RLDMTalon.getMotorOutputVoltage());
    // SmartDashboard.putNumber("Rear Left Steer Motor Output Voltage: ", RLSMTalon.getMotorOutputVoltage());
    // //Falcons / Talon FX: Stator Current
    // SmartDashboard.putNumber("Front Left Drive Motor Stator Current: ", FLDMTalon.getStatorCurrent());
    // SmartDashboard.putNumber("Front Left Steer Motor Stator Current: ", FLSMTalon.getStatorCurrent());
    // SmartDashboard.putNumber("Front Right Drive Motor Stator Current: ", FRDMTalon.getStatorCurrent());
    // SmartDashboard.putNumber("Front Right Steer Motor Stator Current: ", FRSMTalon.getStatorCurrent());
    // SmartDashboard.putNumber("Rear Right Drive Motor Stator Current: ", RRDMTalon.getStatorCurrent());
    // SmartDashboard.putNumber("Rear Right Steer Motor Stator Current: ", RRSMTalon.getStatorCurrent());
    // SmartDashboard.putNumber("Rear Left Drive Motor Stator Current: ", RLDMTalon.getStatorCurrent());
    // SmartDashboard.putNumber("Rear Left Steer Motor Stator Current: ", RLSMTalon.getStatorCurrent());
    // //Falcons / Talon FX: Supply Current
    // SmartDashboard.putNumber("Front Left Drive Motor Supply Current: ", FLDMTalon.getSupplyCurrent());
    // SmartDashboard.putNumber("Front Left Steer Motor Supply Current: ", FLSMTalon.getSupplyCurrent());
    // SmartDashboard.putNumber("Front Right Drive Motor Supply Current: ", FRDMTalon.getSupplyCurrent());
    // SmartDashboard.putNumber("Front Right Steer Motor Supply Current: ", FRSMTalon.getSupplyCurrent());
    // SmartDashboard.putNumber("Rear Right Drive Motor Supply Current: ", RRDMTalon.getSupplyCurrent());
    // SmartDashboard.putNumber("Rear Right Steer Motor Supply Current: ", RRSMTalon.getSupplyCurrent());
    // SmartDashboard.putNumber("Rear Left Drive Motor Supply Current: ", RLDMTalon.getSupplyCurrent());
    // SmartDashboard.putNumber("Rear Left Steer Motor Supply Current: ", RLSMTalon.getSupplyCurrent());
    // //Pigeon
    // SmartDashboard.putNumber("Pigeon 2.0 Yaw: ", pigeon2.getYaw());
    // SmartDashboard.putNumber("Pigeon 2.0 Roll: ", pigeon2.getRoll());
    // SmartDashboard.putNumber("Pigeon 2.0 Pitch: ", pigeon2.getPitch());
    // SmartDashboard.putNumber("Pigeon 2.0 Temp: ", pigeon2.getTemp());
    // SmartDashboard.putNumber("Pigeon 2.0 UpTime: ", pigeon2.getUpTime());
    // SmartDashboard.putNumber("Pigeon 2.0 Absolute Compass Heading: ", pigeon2.getAbsoluteCompassHeading());
    // SmartDashboard.putNumber("Pigeon 2.0 Compass Field Strength: ", pigeon2.getCompassFieldStrength());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.disabled();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // DriverStation.reportWarning("Autonomous Init start: " + DriverStation.getMatchTime(), false);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      //m_robotContainer.hack();
      // m_autonomousCommand = Commands.runOnce(() -> DriverStation.reportWarning("Before starting: " + DriverStation.getMatchTime(), false)).andThen(m_autonomousCommand);
      m_autonomousCommand.schedule();
    }
    // DriverStation.reportWarning("Autonomous Init end: " + DriverStation.getMatchTime(), false);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      m_robotContainer.hack();
    }
    //driveSubsystem.unPark();
  }
//  private final CANSparkMax intakeNeo = new CANSparkMax(1, MotorType.kBrushless);
  
  /** This function is called periodically during operator control. */

  @Override
  public void teleopPeriodic() {
    //while(Constants.mXboxController.getRightTriggerAxis() > 0.50 && Constants.mXboxController.getLeftTriggerAxis() > 0.50){
      // TalonFX lowerMaster = new TalonFX(9, "CANivore");
      // TalonFX lowerSlave = new TalonFX(11, "CANivore");
      // TalonFX highMaster = new TalonFX(8, "CANivore");
      // TalonFX highSlave = new TalonFX(10, "CANivore");
      // lowerSlave.follow(lowerMaster);
      // lowerSlave.setInverted(true);

      // highSlave.follow(highMaster);
      // highSlave.setInverted(true);
      // lowerMaster.set(TalonFXControlMode.PercentOutput, Constants.mXboxController.getLeftY() * 0.25);
      // highMaster.set(TalonFXControlMode.PercentOutput, Constants.mXboxController.getRightY() * 0.25);
    //}
    
    // if(Constants.dXboxController.getLeftTriggerAxis() > 0.5){
    //   intakeNeo.setIdleMode(IdleMode.kBrake);
    //   intakeNeo.set(Constants.dXboxController.getLeftTriggerAxis() * -0.5);
    // } else if (Constants.dXboxController.getRightTriggerAxis() > 0.5){
    //   intakeNeo.setIdleMode(IdleMode.kBrake);
    //   intakeNeo.set(Constants.dXboxController.getRightTriggerAxis() * 0.5);
    // } else if (Constants.dXboxController.getRightTriggerAxis() < 0.1 && Constants.dXboxController.getLeftTriggerAxis() < 0.1){
    //   intakeNeo.set(-.08);
    //   intakeNeo.setIdleMode(IdleMode.kBrake);
    // } 
    // else if(Constants.dXboxController.getLeftTriggerAxis() < 0.1){
    //   intakeNeo.set(0);
    //   intakeNeo.setIdleMode(IdleMode.kBrake);
    // }

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

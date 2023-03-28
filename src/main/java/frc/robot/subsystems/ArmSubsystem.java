// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.EnumMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Dashboard;

public class ArmSubsystem extends SubsystemBase {
  // Controls whether or not to update SmartDashboard
  private boolean armDebug = false;

  // Controls whether or not to move arm to manual positions
  private boolean armDirectDebug = false;

  // Abstraction of the encode positions for a defined arm position
  private class ArmPosition {
    double lowerArmAngle;
    double upperArmAngle;

    public ArmPosition(double lower, double upper) {
      lowerArmAngle = lower;
      upperArmAngle = upper;
    }
  }

  // Enumeration of all defined arm positions
  public enum Position {
    NONE,
    CALIBRATION,
    STARTING,
    PRECHASSIS,
    CHASSIS,
    SAFE,
    GRID_LOW,
    GRID_MID,
    GRID_HIGH,
    INTAKE_PREGROUND,
    INTAKE_GROUND,
    INTAKE_SUBSTATION,
  };

  // Next positions for the Arm to target
  private Deque<Position> targetPosition = new ArrayDeque<Position>();
  private Position lastPosition = Position.STARTING;

  // Lookup "table" for each defined arm position
  private final EnumMap<Position, ArmPosition> armPositions = new EnumMap<>(Map.of(
      // All positions are lower arm first, upper arm second
      // Zero angles are with lower arm vertical and upper arm horizontal
      Position.STARTING, new ArmPosition(-20, -58),
      Position.PRECHASSIS, new ArmPosition(-32, -45),
      Position.CHASSIS, new ArmPosition(-20, -48),
      Position.SAFE, new ArmPosition(-19, 11),
      Position.GRID_LOW, new ArmPosition(-10, -43),
      Position.GRID_MID, new ArmPosition(-5, -8),
      Position.GRID_HIGH, new ArmPosition(28, 22),
      Position.INTAKE_PREGROUND, new ArmPosition(6, -50),
      Position.INTAKE_GROUND, new ArmPosition(11, -72),
      Position.INTAKE_SUBSTATION, new ArmPosition(-16, 14)
    ));

  private LED s_Led;
  private IntakeSubsystem s_Intake;

  private WPI_TalonFX lowerArmMaster;
  private WPI_TalonFX lowerArmSlave;

  private WPI_TalonFX upperArmMaster;
  private WPI_TalonFX upperArmSlave;

  private CANCoder lowerArmCoder;
  private CANCoder upperArmCoder;

  private ProfiledPIDController lowerPidController;
  private ProfiledPIDController upperPidController;

  private final PIDConstants lowerConstants = new PIDConstants(0.70, 0.00, 0.00);
  private final PIDConstants upperConstantsTeleOp = new PIDConstants(0.25, 0.00, 0.00);
  private final PIDConstants upperConstantsAuto = new PIDConstants(0.34, 0.00, 0.00);
  private final TrapezoidProfile.Constraints lowerConstraints = new TrapezoidProfile.Constraints(180, 270);
  private final TrapezoidProfile.Constraints upperConstraints = new TrapezoidProfile.Constraints(180, 270);
  private boolean teleOpInit = false;

  private double lowerMaxVoltage = 12;
  private double upperMaxVoltage = 12;

  private boolean followingWaypoint = false;

  public ArmSubsystem(LED s_Led, IntakeSubsystem s_Intake) {
    this.s_Led = s_Led;
    this.s_Intake = s_Intake;

    // Note that Map.of() only supports 10 key-value pairs, so calibration here
    armPositions.put(Position.CALIBRATION, new ArmPosition(-20, 0));

    this.lowerArmMaster = new WPI_TalonFX(Constants.Arm.lowerMaster, Constants.CANivoreName);
    this.lowerArmSlave = new WPI_TalonFX(Constants.Arm.lowerSlave, Constants.CANivoreName);
    this.lowerArmCoder = new CANCoder(Constants.Arm.lowCoder, Constants.CANivoreName);

    this.upperArmMaster = new WPI_TalonFX(Constants.Arm.upperMaster, Constants.CANivoreName);
    this.upperArmSlave = new WPI_TalonFX(Constants.Arm.upperSlave, Constants.CANivoreName);
    this.upperArmCoder = new CANCoder(Constants.Arm.upperCoder, Constants.CANivoreName);

    this.lowerPidController = new ProfiledPIDController(lowerConstants.kP, lowerConstants.kI, lowerConstants.kD, lowerConstraints);
    this.upperPidController = new ProfiledPIDController(upperConstantsAuto.kP, upperConstantsAuto.kI, upperConstantsAuto.kD, upperConstraints);

    lowerPidController.disableContinuousInput();
    upperPidController.disableContinuousInput();

    lowerArmSlave.follow(lowerArmMaster);
    lowerArmSlave.setInverted(true);
    lowerArmMaster.setInverted(false);
    upperArmSlave.follow(upperArmMaster);
    upperArmSlave.setInverted(false);
    upperArmMaster.setInverted(true);

    SmartDashboard.putData("Coast arm motors", Commands.startEnd(this::coast, this::brake, this));

    Dashboard.watchBoolean("Arm debug", armDebug, (val) -> armDebug = val.booleanValue());
    Dashboard.watchBoolean("Arm direct debug", armDirectDebug, (val) -> armDirectDebug = val.booleanValue());

    brake();
  }

  public boolean isDirectMode() {
    return armDirectDebug;
  }

  private void brake() {
    lowerArmMaster.setNeutralMode(NeutralMode.Brake);
    upperArmMaster.setNeutralMode(NeutralMode.Brake);
    lowerArmSlave.setNeutralMode(NeutralMode.Brake);
    upperArmSlave.setNeutralMode(NeutralMode.Brake);
    lowerArmMaster.stopMotor();
    upperArmMaster.stopMotor();
  }

  private void coast() {
    lowerArmMaster.setNeutralMode(NeutralMode.Coast);
    upperArmMaster.setNeutralMode(NeutralMode.Coast);
    lowerArmSlave.setNeutralMode(NeutralMode.Coast);
    upperArmSlave.setNeutralMode(NeutralMode.Coast);
  }

  // Limit a value (positive or negative)
  private double constrainValue(double value, double max) {
    return Math.signum(value) * Math.min(Math.abs(value), max);
  }

  private double normalizeAngle(double angle) {
    return (angle + 180) % 360 - 180;
  }

  public double getLowerAngle() {
    return normalizeAngle(lowerArmCoder.getAbsolutePosition() - Constants.Arm.lowerOffset + armPositions.get(Position.CALIBRATION).lowerArmAngle);
  }
  public double getUpperAngle() {
    return normalizeAngle(upperArmCoder.getAbsolutePosition() - Constants.Arm.highOffset + armPositions.get(Position.CALIBRATION).upperArmAngle);
  }

  public double getLowerAbsoluteAngle(){
    return lowerArmCoder.getAbsolutePosition();
  }

  public double getUpperAbsoluteAngle(){
    return upperArmCoder.getAbsolutePosition();
  }

  public void setTargetPosition(Position position) {
    Position lastTarget = targetPosition.isEmpty() ? null : targetPosition.getFirst();
    
    targetPosition.clear();
    
    if (!armDirectDebug && lastPosition != position && lastTarget != position) {
      // Determine first step based on last known position
      switch (lastPosition) {
        case STARTING:
          targetPosition.add(Position.PRECHASSIS);
        case PRECHASSIS:
        case CHASSIS:
          targetPosition.add(Position.CHASSIS);
          break;
        case GRID_HIGH:
        case SAFE:
          targetPosition.add(Position.SAFE);
          break;
        case INTAKE_PREGROUND:
        case INTAKE_GROUND:
          targetPosition.add(Position.INTAKE_PREGROUND);
          break;
        case GRID_LOW:
        case GRID_MID:
        case INTAKE_SUBSTATION:
        case CALIBRATION:
        case NONE:
          break;
      }

      // Now we know we are transitioning from a more limited set of positions
      // Determine any prequisites for final position
      switch (position) {
        case PRECHASSIS:
        case STARTING:
          if (targetPosition.isEmpty() || targetPosition.getLast() != Position.CHASSIS) {
            targetPosition.add(Position.CHASSIS);
            if (position == Position.STARTING) {
              targetPosition.add(Position.PRECHASSIS);
            }
          }
          break;
        case INTAKE_GROUND:
        case INTAKE_PREGROUND:
          if (targetPosition.isEmpty() || targetPosition.getLast() != Position.INTAKE_PREGROUND) {
            targetPosition.add(Position.INTAKE_PREGROUND);
          }
          break;
        case GRID_MID:
        case GRID_HIGH:
          if (targetPosition.isEmpty() || targetPosition.getLast() != Position.SAFE) {
            targetPosition.add(Position.SAFE);
          }
          break;
        case INTAKE_SUBSTATION:
        case CHASSIS:
        case GRID_LOW:
        case SAFE:
          // Safe directly from any of the previous positions
          break;
        case CALIBRATION:
        case NONE:
          break;  
      }
    }
    if (targetPosition.isEmpty() || targetPosition.getLast() != position) {
      targetPosition.add(position);
    }

    // Show what position the arm is moving to
    switch (position) {
      case GRID_LOW:
        s_Led.setColor(LED.Colors.blue, 0.0, 20.0);
      case GRID_MID:
        s_Led.setColor(LED.Colors.blue, 0.0, 50.0);
      case GRID_HIGH:
        s_Led.setColor(LED.Colors.blue, 0.0, 80.0);
      default:
        s_Intake.setColor();
        break;
    }
  }

  private boolean setupWaypoint(State initialUpper, double goalUpper, double wayptUpper,
                                State initialLower, double goalLower, double wayptLower) {
    int directWayptUpper = initialUpper.position > wayptUpper ? -1 : 1;
    int directGoalUpper = wayptUpper > goalUpper.position ? -1 : 1;
    if (directWayptUpper != directGoalUpper) {
      // Don't handle a flip of a direction as a waypoint; use as a separate ProfilePID altogether
      DriverStation.reportWarning("Cannot use waypoint, because upper arm reverses", false);
      return false;
    }

    int directWayptLower = initialLower.position > wayptLower ? -1 : 1;
    int directGoalLower = wayptLower > goalLower.position ? -1 : 1;
    if (directWayptLower != directGoalLower) {
      // Don't handle a flip of a direction as a waypoint; use as a separate ProfilePID altogether
      DriverStation.reportWarning("Cannot use waypoint, because lower arm reverses", false);
      return false;
    }

    if (directWayptUpper < 0) {
      initialUpper.position *= -1;
      initialUpper.velocity *= -1;
      wayptUpper *= -1;
      goalUpper *= -1;
    }

    if (directWayptLower < 0) {
      initialUpper.position *= -1;
      initialUpper.velocity *= -1;
      wayptUpper *= -1;
      goalUpper *= -1;
    }

    if (Math.abs(initialUpper.velocity) > upperConstraints.maxVelocity) {
      initialUpper.velocity = upperConstraints.maxVelocity * Math.signum(initialUpper.velocity);
    }

    if (Math.abs(initialLower.velocity) > lowerConstraints.maxVelocity) {
      initialLower.velocity = lowerConstraints.maxVelocity * Math.signum(initialLower.velocity);
    }

/*
    // Deal with a possibly truncated motion profile (with nonzero initial or
    // final velocity) by calculating the parameters as if the profile began and
    // ended at zero velocity
    double cutoffBegin = m_initial.velocity / m_constraints.maxAcceleration;
    double cutoffDistBegin = cutoffBegin * cutoffBegin * m_constraints.maxAcceleration / 2.0;

    double cutoffEnd = m_goal.velocity / m_constraints.maxAcceleration;
    double cutoffDistEnd = cutoffEnd * cutoffEnd * m_constraints.maxAcceleration / 2.0;

    // Now we can calculate the parameters as if it was a full trapezoid instead
    // of a truncated one

    double fullTrapezoidDist =
        cutoffDistBegin + (m_goal.position - m_initial.position) + cutoffDistEnd;
    double accelerationTime = m_constraints.maxVelocity / m_constraints.maxAcceleration;

    double fullSpeedDist =
        fullTrapezoidDist - accelerationTime * accelerationTime * m_constraints.maxAcceleration;

    // Handle the case where the profile never reaches full speed
    if (fullSpeedDist < 0) {
      accelerationTime = Math.sqrt(fullTrapezoidDist / m_constraints.maxAcceleration);
      fullSpeedDist = 0;
    }
*/

    // Calculate distance to cover to waypoint
    double upperDist = wayptUpper - initialUpper.position;
    double lowerDist = wayptLower - initialLower.position;

    // Change in velocity over acceleration gives time to make that change
    double upperAccelTime = (upperConstraints.maxVelocity - initialUpper.velocity) / upperConstraints.maxAcceleration;
    double lowerAccelTime = (lowerConstraints.maxVelocity - initialLower.velocity) / lowerConstraints.maxAcceleration;

    // Distance covered while accelerating
    // TODO Presume initial velocity is 0
    // Distance = Acceleration * Time^2 / 2
    double upperAccelDist = upperConstraints.maxAcceleration * upperAccelTime * upperAccelTime / 2.0;
    double lowerAccelDist = lowerConstraints.maxAcceleration * lowerAccelTime * lowerAccelTime / 2.0;

    State wayptUpperState = new State(wayptUpper, 0.0);
    State wayptLowerState = new State(wayptLower, 0.0);

    double upperWayptCVTime = 0.0;
    double lowerWayptCVTime = 0.0;

    // Not all calcuations currently handle a non-zero starting velocity
    if (initialUpper.velocity != 0 || initialLower.velocity != 0) {
      throw new Throwable("TODO Support calculations with a non-zero starting velocity");
    }

    if (upperAccelDist > upperDist) {
      DriverStation.reportWarning("Waypoint will be reached before full acceleration", false);
      // TODO Technically, if the back half is even shorter, maybe we shouldn't even fully accelerate -- but ignore for now
      upperAccelTime = Math.sqrt(upperDist / upperConstraints.maxAcceleration * 2.0); // TODO Presumes initialUpper.velocity == 0
      wayptUpperState.velocity = initialUpper.velocity + upperAccelTime * upperConstraints.maxAcceleration;
    } else {
      // We will fully accelerate to max velocity
    }
    } else if((cutoffDistBegin + cutoffDistEnd) > (m_intermediate.position - m_initial.position)) {
      // TODO Handle intermediate positon reached during deceleration period
      DriverStation.reportWarning("Intermediate position will be reached without full acceleration and deceleration", false);
    } else {
      // We have time to fully accelerate
      upperWayptCVTime = (upperDist - upperAccelDist) / upperConstraints.maxVelocity;
    }

    double wayptTimeUpper = upperAccelTime + upperWayptCVTime;

    TrapezoidProfile.Constraints upperWayptConstraints = new TrapezoidProfile.Constraints(upperConstraints.maxVelocity, upperConstraints.maxAcceleration);

      if (wayptTimeLower > wayptTimeUpper) {
        wayptTime = wayptTimeLower;
        constraintsUpper.maxVelocity = intermediateUpper.velocity = requiredVelocityAfterRamp(upperConstraints.maxAcceleration, upperDist, wayptTime);
      } else {
        wayptTime = wayptTimeUpper;
        constraintsLower.maxVelocity = intermediateLower.velocity = requiredVelocityAfterRamp(upperConstraints.maxAcceleration, lowerDist, wayptTime);
      }

      wayptPIDUpper.setConstraints(constraintsUpper);
      wayptPIDLower.setConstraints(constraintsLower);

      wayptPIDUpper.reset(upperInitial);
      wapptPIDLower.reset(lowerInitial);
    }

    m_endAccel = accelerationTime - cutoffBegin;
    m_endFullSpeed = m_endAccel + fullSpeedDist / m_constraints.maxVelocity;
    m_endDeccel = m_endFullSpeed + accelerationTime - cutoffEnd;
  }

  // Returns velocity to covert a distance in set amount of time, given need
  // to accelerate first given the specified acceleration
  // NOTE: Presumes no starting velocity
  private requiredVelocityAfterRamp(double accel, double distance, double time) {
    // Derived that v^2 / (2*a) + (t - v/a)*v = d
    // This is the (useful) quadratic solution: a*t - sqrt(a^2*t^2 - 2*a*d)
    return accel * time - sqrt(accel * accel * time * time - 2.0 * accel * distance);
  }

  public void beginMovement() {
    ArmPosition nextPosition = armPositions.get(targetPosition.peek());

    // TODO Consider passing a State, so that velocity can be non-zero for intermediate points
    lowerPidController.setGoal(nextPosition.lowerArmAngle);
    upperPidController.setGoal(nextPosition.upperArmAngle);
    if (!teleOpInit && DriverStation.isTeleop()) {
      upperPidController.setPID(upperConstantsTeleOp.kP, upperConstantsTeleOp.kI, upperConstantsTeleOp.kD);
      teleOpInit = true;
    }
    if (targetPosition.size() > 1) {
      if (!followingWaypoint && targetPosition.peek() == Position.SAFE) {
        lowerPidController.setTolerance(25);
        upperPidController.setTolerance(25);
      } else {
        lowerPidController.setTolerance(5, 15);
        upperPidController.setTolerance(5, 15);
      }
    } else {
      lowerPidController.setTolerance(2, 5);
      upperPidController.setTolerance(2, 5);
    }
    if (targetPosition.size() == 2) {
      // TODO Insert waypoint logic
      Position goalPosition = targetPosition.peekLast();
      followingWaypoint = setupWaypoint(getUpperAngle(), goalPosition.upperArmAngle, nextPosition.upperArmAngle,
                                        getLowerAngle(), goalPosition.lowerArmAngle, nextPosition.lowerArmAngle);
    } else {
      if (followingWaypoint) { // TODO When are we continuing, vs when are we a fresh target?
        // TODO Logic to progress past waypoint
        assert(targetPosition.size() == 1);
        lowerPidController.reset(getLowerAngle(), lowerPidController.getGoal().maxVelocity); // TODO Might not have been at max?
        upperPidController.reset(getUpperAngle(), upperPidController.getGoal().maxVelocity);
      } else {
        // TODO Can this be made smoother if we can acknowledge a non-zero starting velocity?
        lowerPidController.reset(getLowerAngle());
        upperPidController.reset(getUpperAngle());
      }
      followingWaypoint = false;
    }
  }

  public void updateMovement() {
    if (lowerPidController.atGoal() && upperPidController.atGoal()) {
      lastPosition = targetPosition.remove();
      if (!doneMovement()) {
        beginMovement();
      }
    }
    if (!doneMovement()) {
      // TODO Consider removing (or considerably raising) max voltage with PID profile in place
      lowerArmMaster.setVoltage(constrainValue(lowerPidController.calculate(getLowerAngle()), lowerMaxVoltage));
      upperArmMaster.setVoltage(constrainValue(upperPidController.calculate(getUpperAngle()), upperMaxVoltage));
    } else {
      stopMovement();
    }
  }

  public boolean doneMovement() {
    return targetPosition.isEmpty();
  }

  public void stopMovement() {
    lowerArmMaster.stopMotor();
    upperArmMaster.stopMotor();
  }

  public void lowerArmDirect(double voltage) {
    lowerArmMaster.setVoltage(voltage);
    if (voltage == 0) {
      lowerArmMaster.stopMotor();
    }
  }

  public void upperArmDirect(double voltage) {
    upperArmMaster.setVoltage(voltage);
    if (voltage == 0) {
      upperArmMaster.stopMotor();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update dashboard to help monitor and debug

    if (armDebug) {
      SmartDashboard.putNumber("Target queue depth", targetPosition.size());
      SmartDashboard.putString("Target Position", targetPosition.isEmpty() ? "<none>" : targetPosition.peek().toString());
      SmartDashboard.putNumber("Lower Arm goal", lowerPidController.getGoal().position);
      SmartDashboard.putNumber("Upper Arm goal", upperPidController.getGoal().position);
      SmartDashboard.putBoolean("Lower Arm set? ", lowerPidController.atGoal());
      SmartDashboard.putBoolean("Upper Arm set? ", upperPidController.atGoal());

      SmartDashboard.putNumber("Lower Angle ABSOLUTE", getLowerAbsoluteAngle());
      SmartDashboard.putNumber("Upper Angle ABSOLUTE", getUpperAbsoluteAngle());
      SmartDashboard.putNumber("Lower Angle", getLowerAngle());
      SmartDashboard.putNumber("Upper Angle", getUpperAngle());

      SmartDashboard.putNumber("Lower Arm Master Voltage:", lowerArmMaster.getMotorOutputVoltage());
      SmartDashboard.putNumber("Lower Arm Slave Voltage:", lowerArmSlave.getMotorOutputVoltage());

      SmartDashboard.putNumber("Upper Arm Master Voltage:", upperArmMaster.getMotorOutputVoltage());
      SmartDashboard.putNumber("Upper Arm Slave Voltage:", upperArmSlave.getMotorOutputVoltage());
    }
  }
}
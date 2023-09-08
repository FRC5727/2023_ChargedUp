package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.lib.math.BetterSwerveModuleState;
import frc.lib.math.GeometryUtils;
import frc.lib.math.SecondOrderSwerveModuleStates;
import frc.robot.Constants;
import frc.robot.Dashboard;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public static boolean swerveDebug = false;
    private boolean speedLimit = false;

    double previousT;
    double offT;
    Timer timer = new Timer();

    private Rotation2d targetHeading;
    
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.CANivoreName);
        gyro.configFactoryDefault();
        zeroGyro();

        Dashboard.watchBoolean("Swerve debug", swerveDebug, (val) -> swerveDebug = val.booleanValue());

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();
        timer.start();
    }
    private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
        final double LOOP_TIME_S = 0.02; 
        Pose2d futureRobotPose =
            new Pose2d(
                originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
                originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
                Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));   
        Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
        ChassisSpeeds updatedSpeeds =
            new ChassisSpeeds(
                twistForPose.dx / LOOP_TIME_S,
                twistForPose.dy / LOOP_TIME_S,
                twistForPose.dtheta / LOOP_TIME_S);
        return updatedSpeeds;  
    }
    private ChassisSpeeds correctHeading(ChassisSpeeds desiredSpeed){
        //Determine time interval
        double currentT = timer.get();
        double dt = currentT - previousT;
        //Get desired rotational speed in radians per second and absolute translational speed in m/s
        double vr = desiredSpeed.omegaRadiansPerSecond;
        double v = Math.hypot(desiredSpeed.vxMetersPerSecond, desiredSpeed.vyMetersPerSecond);
        if (vr > 0.01 || vr < -0.01){
            offT = currentT;
            setTargetHeading(getYaw());
            return desiredSpeed;
        }
        if (currentT - offT < 0.5){
            setTargetHeading(getYaw());
            return desiredSpeed;
        }
        //Determine target and current heading
        setTargetHeading( getTargetHeading().plus(new Rotation2d(vr * dt)) );
        Rotation2d currentHeading = getYaw();
        //Calculate the change in heading that is needed to achieve the target
        Rotation2d deltaHeading = getTargetHeading().minus(currentHeading);
        if (Math.abs(deltaHeading.getDegrees()) < 0.05){
            return desiredSpeed;
        }
        double correctedVr = deltaHeading.getRadians() / dt * 0.05;
        previousT = currentT;

        return new ChassisSpeeds(desiredSpeed.vxMetersPerSecond, desiredSpeed.vyMetersPerSecond, correctedVr);
    }
    public Rotation2d getTargetHeading(){ 
        return targetHeading; 
    }
    public void setTargetHeading(Rotation2d targetHeading) { 
        this.targetHeading = targetHeading; 
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {        
        ChassisSpeeds desiredChassisSpeeds =
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(),
            translation.getY(),
            rotation,
            getYaw())
            : new ChassisSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation);
        desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);
        
        desiredChassisSpeeds = correctHeading(desiredChassisSpeeds);
        
        SecondOrderSwerveModuleStates secondOrderSwerveModuleStates = Constants.Swerve.secondKinematics.toSwerveModuleState(desiredChassisSpeeds, getYaw());
        SwerveModuleState[] swerveModuleStates = secondOrderSwerveModuleStates.getSwerveModuleStates();
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        
        // BetterSwerveModuleState[] swerveModuleStates = Constants.Swerve.betterKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);        

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }
    public void stopDrive() {
        drive(new Translation2d(0, 0), 0, false, true);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw())
                : Rotation2d.fromDegrees(gyro.getYaw());
    }
    public Rotation2d getFalseYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw() + 360)
                : Rotation2d.fromDegrees(gyro.getYaw() + 180);
    }

    // TODO Remove
    public void hack() {
        gyro.setYaw(gyro.getYaw() + 180);
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public void enableSpeedLimit() {
        speedLimit = true;
    }

    public void disableSpeedLimit() {
        speedLimit = false;
    }

    public double getSpeedLimitXY() {
        return speedLimit ? Constants.Swerve.speedLimitXY : 1.0;
    }

    public double getSpeedLimitRot() {
        return speedLimit ? Constants.Swerve.speedLimitRot : 1.0;
    }

    public double getPitch(){
        // The gyro is rotated 90 degrees, so the gyro roll is the robot pitch
        return gyro.getRoll();
    }

    @Override
    public void periodic() {
        if (swerveDebug) {
            for (SwerveModule mod : mSwerveMods) {
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            }
            SmartDashboard.putNumber("Gyro Angle", getYaw().getDegrees());
            SmartDashboard.putNumber("Robot Pitch", getPitch());
            // SmartDashboard.putNumber("Gyro Roll", gyro.getRoll());
        }
    }
}

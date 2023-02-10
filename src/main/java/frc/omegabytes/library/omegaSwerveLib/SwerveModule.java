package frc.omegabytes.library.omegaSwerveLib;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    double getPosition();

    void set(double driveVoltage, double steerAngle);
}

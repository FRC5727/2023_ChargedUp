package frc.omegabytes.library.omegaSwerveLib;

public interface DriveController {
    void setReferenceVoltage(double voltage);

    double getStateVelocity();

    double getPosition();
}

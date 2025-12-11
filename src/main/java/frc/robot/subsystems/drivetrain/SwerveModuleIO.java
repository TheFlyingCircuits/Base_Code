package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    
    @AutoLog
    public class SwerveModuleIOInputs {
        public double drivePositionMeters = 0.0;
        public double driveVelocityMetersPerSecond = 0.0;
        
        public double angleAbsolutePositionDegrees = 0.0;
        public double targetAngleDegrees = 0.0;

        public double driveAppliedVoltage = 0.0;
        public double driveCurrent = 0.0;
    }
    
    public default void updateInputs(SwerveModuleIOInputs inputs) {};

    public default void setDriveVelocity(double velocityMetersPerSecond) {};

    public default void setTurnAngle(double angleDegrees) {};

    public default void setDriveVoltage(double volts) {};
    
    public default void setAngleVoltage(double volts) {};


}

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import frc.robot.TunerConstants;
import frc.robot.VendorWrappers.Kraken;

public class SwerveModuleIOKraken implements SwerveModuleIO {

    private CANcoder absoluteEncoder;
    private Kraken steerKraken;
    private Kraken driveKraken;

    private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> driveConstants;

    public SwerveModuleIOKraken(SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.driveConstants=constants;

        absoluteEncoder = new CANcoder(constants.EncoderId, TunerConstants.DrivetrainConstants.CANBusName);
        steerKraken = new Kraken(constants.SteerMotorId, TunerConstants.DrivetrainConstants.CANBusName);
        driveKraken = new Kraken(constants.DriveMotorId, TunerConstants.DrivetrainConstants.CANBusName);

        TalonFXConfiguration steerConfig = constants.DriveMotorInitialConfigs;
        TalonFXConfiguration driveConfig = constants.DriveMotorInitialConfigs;

        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerConfig.Slot0 = constants.SteerMotorGains;
        steerConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
        steerConfig.Feedback.FeedbackSensorSource =
        switch (constants.FeedbackSource) {
          case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
          case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
          case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
          default ->
              throw new RuntimeException(
                  "You have selected a turn feedback source that is not supported by the default implementation of ModuleIOTalonFX. Please check the AdvantageKit documentation for more information on alternative configurations: https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations");
        };
        steerConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
        steerConfig.MotorOutput.Inverted =
            constants.SteerMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        steerKraken.getConfigurator().apply(steerConfig);

        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0 = constants.DriveMotorGains;
        driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.Inverted =
            constants.DriveMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        driveKraken.getConfigurator().apply(driveConfig);
        driveKraken.setPosition(0.0);

        CANcoderConfiguration cancoderConfig = constants.EncoderInitialConfigs;
        cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
        cancoderConfig.MagnetSensor.SensorDirection =
            constants.EncoderInverted
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;
        absoluteEncoder.getConfigurator().apply(cancoderConfig);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionMeters = driveKraken.getPosition().getValueAsDouble() * ((1 / driveConstants.DriveMotorGearRatio) * (2 * Math.PI * driveConstants.WheelRadius));
        inputs.driveVelocityMetersPerSecond = driveKraken.getVelocity().getValueAsDouble() * ((1 / driveConstants.DriveMotorGearRatio) * (2 * Math.PI * driveConstants.WheelRadius));
        inputs.angleAbsolutePositionDegrees = absoluteEncoder.getAbsolutePosition().getValueAsDouble()*360;

        inputs.driveAppliedVoltage = driveKraken.getMotorVoltage().getValueAsDouble();
        inputs.driveCurrent = driveKraken.getTorqueCurrent().getValueAsDouble();
    }

    @Override
    public void setDriveVelocity(double velocityMetersPerSecond) {
        double velocityRotationsPerSecond = velocityMetersPerSecond/((1 / driveConstants.DriveMotorGearRatio) * (2 * Math.PI * driveConstants.WheelRadius));
        driveKraken.setControl(velocityTorqueCurrentRequest.withVelocity(velocityRotationsPerSecond));
        System.out.println("this is being called");
    }

    @Override
    public void setTurnAngle(double angleRotations) {
        steerKraken.setControl(positionTorqueCurrentRequest.withPosition(angleRotations));
    }
    
}

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.UniversalConstants;
import frc.robot.VendorWrappers.Kraken;

public class SwerveModuleIOKraken implements SwerveModuleIO {

    private double desiredAngleDeg = 0.0;


    final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
    final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(1);

     private CANcoder absoluteEncoder;
    private Kraken angleMotor;
    private Kraken driveMotor;


    public SwerveModuleIOKraken(int driveMotorID, int angleMotorID, double angleOffsetDegrees, int cancoderID, String name) {
        this(driveMotorID, angleMotorID, angleOffsetDegrees, cancoderID, false, false, name);
    }

    /**
     * 
     * @param driveMotorID - ID of the drive motor
     * @param angleMotorID - ID of the angle motor
     * @param angleOffsetDegrees - Offset of the angle motor, in degrees
     * @param cancoderID - ID of the absolute CANcoder mounted ontop of the swerve
     * @param isDriveMotorOnTop - Is drive motor mounted on top
     * @param isAngleMotorOnTop - Is angle motor mounted on top
     */
    public SwerveModuleIOKraken(int driveMotorID, int angleMotorID, double angleOffsetDegrees, int cancoderID, boolean isDriveMotorOnTop, boolean isAngleMotorOnTop, String name){
        /* Angle Encoder Config */
        absoluteEncoder = new CANcoder(cancoderID, UniversalConstants.canivoreName);
        configCANCoder(angleOffsetDegrees);

        /* Angle Motor Config */
        angleMotor = new Kraken(name+"Steer", angleMotorID, UniversalConstants.canivoreName);
        if(isAngleMotorOnTop) {
            configAngleMotor(InvertedValue.CounterClockwise_Positive,cancoderID);
        } else {
            configAngleMotor(InvertedValue.Clockwise_Positive,cancoderID);
        }

        /* Drive Motor Config */
        driveMotor = new Kraken(name+"Drive", driveMotorID, UniversalConstants.canivoreName);
        if(isDriveMotorOnTop) {
            configDriveMotor(InvertedValue.CounterClockwise_Positive);
        } else {
            configDriveMotor(InvertedValue.Clockwise_Positive);
        }
    }

    private void configCANCoder(double angleOffsetDegrees) {
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        cancoderConfigs.MagnetSensor.MagnetOffset = angleOffsetDegrees;
        cancoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        absoluteEncoder.getConfigurator().apply(cancoderConfigs);
    }

    private void configDriveMotor(InvertedValue invertedValue) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = invertedValue;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 45; // re-determined after firmware upgrade to prevent wheel slip. Feels pretty low though

        config.Slot1.kS = 0.2383; 
        config.Slot1.kV = 0.12437965961;
        config.Slot1.kP = 0.0;
        config.Slot1.kI = 0.0;
        config.Slot1.kD = 0.0;
        driveMotor.applyConfig(config);
    }

    private void configAngleMotor(InvertedValue invertedValue, int cancoderID) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = invertedValue;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 60;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kS = 0.0;
        config.Slot0.kP = 28.8;
        config.Slot0.kI = 0.0; 
        config.Slot0.kD = 0.0;
        config.Feedback.FeedbackRemoteSensorID = cancoderID;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.Feedback.SensorToMechanismRatio = 1;
        config.ClosedLoopGeneral.ContinuousWrap = true;
        config.TorqueCurrent.withPeakForwardTorqueCurrent(10)
      .withPeakReverseTorqueCurrent(-10);
        angleMotor.applyConfig(config);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionMeters = driveMotor.getPosition().getValueAsDouble() * (SwerveModuleConstants.driveGearReduction * SwerveModuleConstants.wheelCircumferenceMeters);
        inputs.driveVelocityMetersPerSecond = driveMotor.getVelocity().getValueAsDouble() * (SwerveModuleConstants.driveGearReduction * SwerveModuleConstants.wheelCircumferenceMeters);
        inputs.angleAbsolutePositionDegrees = absoluteEncoder.getAbsolutePosition().getValueAsDouble()*360;
        inputs.targetAngleDegrees = desiredAngleDeg;

        inputs.driveAppliedVoltage = driveMotor.getMotorVoltage().getValueAsDouble();
        inputs.driveCurrent = driveMotor.getTorqueCurrent().getValueAsDouble();
    }

    @Override
    public void setDriveVelocity(double velocityMetersPerSecond) {
        double velocityRotationsPerSecondDriveWheels = velocityMetersPerSecond/SwerveModuleConstants.wheelCircumferenceMeters;
        double velocityRotationsPerSecondDriveMotor = velocityRotationsPerSecondDriveWheels/SwerveModuleConstants.driveGearReduction;

        driveMotor.setControl(velocityRequest.withVelocity(velocityRotationsPerSecondDriveMotor));
    }

    @Override
    public void setTurnAngle(double angleDegrees) {
        desiredAngleDeg = angleDegrees;
        angleMotor.setControl(positionRequest.withPosition(Units.degreesToRotations(angleDegrees)));
    }
    
    
    @Override
    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }

    @Override
    public void setAngleVoltage(double volts) {
        angleMotor.setVoltage(volts);
    }
}

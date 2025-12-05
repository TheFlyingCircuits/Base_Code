package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    public int moduleIndex;

    private SwerveModuleIO io;
    private SwerveModuleIOInputsAutoLogged inputs;
    
    private String name;

     public SwerveModule(SwerveModuleIO io, int moduleIndex, String name) {
        this.io = io;
        this.moduleIndex = moduleIndex;
        this.name = name;

        inputs = new SwerveModuleIOInputsAutoLogged();
    }

    public void periodic() {
        io.updateInputs(inputs);

        Logger.processInputs("swerveModuleInputs/"+name, inputs);
    }


    private SwerveModuleState constrainState(SwerveModuleState toConstrain) {
        double originalDegrees = toConstrain.angle.getDegrees();
        double constrainedDegrees = MathUtil.inputModulus(originalDegrees, -180, 180);
        return new SwerveModuleState(toConstrain.speedMetersPerSecond, Rotation2d.fromDegrees(constrainedDegrees));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Make sure the desired state's angle is in the range [-180, 180], so it can be appropriately
        // compared to the current angle from the CANCoder. We verified that you need constrainState()
        // to be called both before and after SwerveModuleState.optimize().
        desiredState = constrainState(desiredState);
        desiredState = SwerveModuleState.optimize(
            desiredState,
            Rotation2d.fromDegrees(inputs.angleAbsolutePositionDegrees)
        );
        // System.out.println(desiredState.angle.getDegrees());

                
        setDesiredStateNoOptimize(desiredState);
    }

    public void setDesiredStateNoOptimize(SwerveModuleState desiredState) {
        
        desiredState = constrainState(desiredState); // constrain one more time after optimization just to be safe, because I'm unsure if optimization can ever pull the angle out of [-180, 180]

        io.setDriveVelocity(desiredState.speedMetersPerSecond);
        io.setTurnAngle(desiredState.angle.getDegrees());
    }


    public SwerveModuleState getState() {

        return new SwerveModuleState(
            inputs.driveVelocityMetersPerSecond,
            Rotation2d.fromDegrees(
                // Use the absolute encoder.
                inputs.angleAbsolutePositionDegrees)
            );
    }



    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            inputs.drivePositionMeters,
            // Use the absolute encoder.
            Rotation2d.fromDegrees(inputs.angleAbsolutePositionDegrees)
        );
        
    }

}

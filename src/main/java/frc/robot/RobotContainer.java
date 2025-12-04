// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.HumanDriver;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIOPigeon;
import frc.robot.subsystems.drivetrain.SwerveModuleIOKraken;
import frc.robot.subsystems.vision.VisionIO;


public class RobotContainer {

    public final Drivetrain drivetrain;

    protected final HumanDriver duncan = new HumanDriver(0);
    final CommandXboxController duncanController;

    public RobotContainer() {
        /**** INITIALIZE SUBSYSTEMS ****/
        // if (RobotBase.isReal()) {
            drivetrain = new Drivetrain( 
                new GyroIOPigeon(),
                new SwerveModuleIOKraken(TunerConstants.FrontLeft), 
                new SwerveModuleIOKraken(TunerConstants.FrontRight),
                new SwerveModuleIOKraken(TunerConstants.BackLeft),
                new SwerveModuleIOKraken(TunerConstants.BackRight),
                new VisionIO() {}
            );
        // }
        duncanController = duncan.getXboxController();
        configureBindings();
        setDefaultCommands();
    }

    private void configureBindings() {}

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public void setDefaultCommands() {
        drivetrain.setDefaultCommand(driverFullyControlDrivetrain().withName("driveDefualtCommand"));
    }

    private Command driverFullyControlDrivetrain() { return drivetrain.run(() -> {
        drivetrain.fieldOrientedDrive(duncan.getRequestedFieldOrientedVelocity());
        Logger.recordOutput("drivetrain/runningDefaultCommand", true);
        }).finallyDo(() -> {
            Logger.recordOutput("drivetrain/runningDefaultCommand", false);
        }).withName("driverFullyControlDrivetrain");
    }

}

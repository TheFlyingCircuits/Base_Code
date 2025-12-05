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
                new SwerveModuleIOKraken(0, 1, -0.377686, 0, "FL"), 
                new SwerveModuleIOKraken(2, 3, 0.397705, 1, "FR"),
                new SwerveModuleIOKraken(4, 5, 0.238281, 2, "BL"),
                new SwerveModuleIOKraken(6, 7,  -0.370850, 3, "BR"),
                new VisionIO() {} //VisionConstants.useNewSingleTagCodeFromBuckeye ? new VisionIO() {} : new VisionIOPhotonLib()
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

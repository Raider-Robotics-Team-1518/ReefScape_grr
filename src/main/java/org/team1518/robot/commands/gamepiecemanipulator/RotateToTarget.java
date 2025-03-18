// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1518.robot.commands.gamepiecemanipulator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1518.robot.LimeLight;
import org.team1518.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://ocs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateToTarget extends Command {

    private final Swerve swerveDrive;
    private final PIDController pidController;
    private final LimeLight limelight;

    /** Creates a new SetTravelPosition. */
    public RotateToTarget(Swerve swerveDrive, LimeLight limelight) {
        this.swerveDrive = swerveDrive;
        this.limelight = limelight;

        pidController = new PIDController(0.02, 0, 0.001);
        pidController.setTolerance(1.0);
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pidController.setSetpoint(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double tx = limelight.getTargetOffsetHorizontal();
        double rotationSpeed = pidController.calculate(tx, 0);
        // swerveDrive.drive(0, 0, rotationSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // swerveDrive.drive(0, 0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }
}

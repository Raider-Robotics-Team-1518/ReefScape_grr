// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1518.robot.commands.gamepiecemanipulator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1518.robot.LimeLight;
import org.team1518.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://ocs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTarget extends Command {

    private final Swerve swerveDrive;
    private final PIDController turnController;
    private final LimeLight limelight;

    private static final double kP = 0.02;
    private static final double kI = 0.0;
    private static final double kD = 0.001;
    private static final double TOLERANCE = 1.0;

    /** Creates a new SetTravelPosition. */
    public AlignToTarget(Swerve swerveDrive, LimeLight limelight) {
        this.swerveDrive = swerveDrive;
        this.limelight = limelight;
        this.turnController = new PIDController(kP, kI, kD);
        turnController.setTolerance(TOLERANCE);

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        turnController.setSetpoint(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double tx = limelight.getTargetOffsetHorizontal();
        double turnSpeed = turnController.calculate(tx, 0);

        swerveDrive.drive(() -> 0, () -> 0, () -> turnSpeed).schedule();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveDrive.drive(() -> 0, () -> 0, () -> 0).schedule();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return turnController.atSetpoint();
    }
}

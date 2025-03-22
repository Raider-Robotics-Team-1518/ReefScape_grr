// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1518.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.DoubleSupplier;
import org.team1518.lib.swerve.Perspective;
import org.team1518.lib.swerve.SwerveAPI;
import org.team1518.robot.LimeLight;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefDrive extends Command {

    private PIDController reefController = new PIDController(0.05, 0.0, 0.005);
    private DoubleSupplier x;
    private DoubleSupplier y;
    private DoubleSupplier angular;
    private SwerveAPI api;
    private LimeLight limeLight;

    /** Creates a new ReefDrive. */
    public ReefDrive(
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        DoubleSupplier angleSupplier,
        Subsystem swerveSubsystem,
        SwerveAPI swervApi
    ) {
        // Use addRequirements() here to declare subsystem dependencies.
        x = xSupplier;
        y = ySupplier;
        angular = angleSupplier;
        api = swervApi;
        limeLight = new LimeLight();
        addRequirements(swerveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        reefController.setSetpoint(0.0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double xInput = x.getAsDouble();
        double yInput = y.getAsDouble();
        double angularInput = angular.getAsDouble();
        boolean discretize = true;
        boolean rateLimit = true;
        Perspective perspective = Perspective.kRobot;

        double vxMetersPerSecond = 0;
        double vyMetersPerSecond = 0;
        double offsetAngle = limeLight.getTargetOffsetHorizontal(); // +- 29 degrees

        //vxMetersPerSecond = offsetAngle / 29;
        vxMetersPerSecond = reefController.calculate(offsetAngle);

        ChassisSpeeds speeds = new ChassisSpeeds(yInput + vyMetersPerSecond, xInput - vxMetersPerSecond, angularInput);
        api.applySpeeds(speeds, perspective, discretize, rateLimit);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

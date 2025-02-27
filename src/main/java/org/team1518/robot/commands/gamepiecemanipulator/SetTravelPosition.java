// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1518.robot.commands.gamepiecemanipulator;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1518.robot.Constants;
import org.team1518.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetTravelPosition extends Command {

    private boolean isDone = false;
    private boolean isAtHeight = false;
    private boolean isAtAngle = false;
    private double current_angle = 0;
    private double current_height = 0;

    /** Creates a new SetTravelPosition. */
    public SetTravelPosition() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.elevatorSubsystem, Robot.wristSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (isAtHeight && isAtAngle) {
            isDone = true;
        } else {
            // set arm to correct angle
            current_angle = Robot.wristSubsystem.getWristPosition();
            // Calculate power curve proportional
            double armRotationPower = Math.abs(Constants.Reef.travelAngle - current_angle) / 100;
            // Move arm up or down to target arm angle
            if (Math.abs(Constants.Reef.travelAngle - current_angle) > Constants.Tolerances.coralIntakeAngleTolerance) {
                double v_sign = Math.signum(Constants.Reef.travelAngle - current_angle);
                Robot.wristSubsystem.setWristSpeed(v_sign * (armRotationPower));
            } else {
                Robot.wristSubsystem.stopWrist();
                isAtAngle = true;
            }
            // set height to correct
            current_height = Robot.elevatorSubsystem.getCurrentHeight();
            // Move elevator up or down to target height
            if (
                Math.abs(Constants.Reef.targetCoralIntakeHeight - current_height) >
                Constants.Tolerances.reefCoralHeightTolerance
            ) {
                double v_sign = Math.signum(Constants.Reef.targetCoralIntakeHeight - current_height);
                Robot.elevatorSubsystem.driveElevator(v_sign * (Constants.MotorSpeeds.elevatorPower));
            } else {
                Robot.elevatorSubsystem.stopElevator();
                isAtHeight = true;
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isDone;
    }
}

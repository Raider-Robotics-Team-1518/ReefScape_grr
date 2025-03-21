// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1518.robot.commands.gamepiecemanipulator;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1518.robot.Constants;
import org.team1518.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetAlgaeTravelPosition extends Command {

    private boolean isDone = false;
    private boolean isAtAngle = false;
    private double current_angle = 0;

    /** Creates a new SetTravelPosition. */
    public SetAlgaeTravelPosition() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.elevatorSubsystem, Robot.wristSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isDone = false;
        isAtAngle = false;
        current_angle = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        current_angle = Robot.wristSubsystem.getWristPosition();
        if (
            Math.abs(Constants.Reef.travelAlgaeAngle - current_angle) <=
            Constants.Tolerances.travelPositionAngleTolerance
        ) {
            Robot.wristSubsystem.stopWrist();
            isDone = true;
        } else {
            // set arm to correct angle
            // Calculate power curve proportional
            double armRotationPower = Math.abs(Constants.Reef.travelAlgaeAngle - current_angle) / 360 + 0.2;
            // Move arm up or down to target arm angle
            double v_sign = Math.signum(Constants.Reef.travelAlgaeAngle - current_angle);
            Robot.wristSubsystem.setWristSpeed(v_sign * (armRotationPower));
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.wristSubsystem.stopWrist();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isDone;
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1518.robot.commands.gamepiecemanipulator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1518.robot.Constants;
import org.team1518.robot.Robot;

public class IntakeAlgae extends Command {

    private Timer timer;
    private boolean isDone = false;
    private double current_angle = Robot.wristSubsystem.getWristPosition();
    private double targetAlgaeIntakeAngle = 30;

    public IntakeAlgae(double targetAlgaeIntakeAngle) {
        addRequirements(Robot.wristSubsystem, Robot.gamePieceManipulator);
        this.targetAlgaeIntakeAngle = targetAlgaeIntakeAngle;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer = new Timer();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        timer.start();
        current_angle = Robot.wristSubsystem.getWristPosition();
        // Calculate power curve proportional
        double armRotationPower = Math.abs(this.targetAlgaeIntakeAngle - current_angle) / 100;
        // Move arm up or down to target arm angle
        if (Math.abs(this.targetAlgaeIntakeAngle - current_angle) > Constants.Tolerances.algaeIntakeAngleTolerance) {
            double v_sign = Math.signum(this.targetAlgaeIntakeAngle - current_angle);
            Robot.wristSubsystem.setWristSpeed(v_sign * (armRotationPower));
        } else {
            // the arm's at the correct angle, stop arm, intake the algae
            Robot.wristSubsystem.setWristSpeed(0d);
            Robot.gamePieceManipulator.intakeAlgae();
            if (timer.hasElapsed(Constants.Times.algaeMotorRunTime)) {
                isDone = true;
                isFinished();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer.stop();
        Robot.gamePieceManipulator.stopAlgaeMotor();;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isDone;
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Class to eject the coral onto the reef
 */

package org.team1518.robot.commands.gamepiecemanipulator;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1518.robot.Constants;
import org.team1518.robot.Robot;

public class MoveToEjectCoralAngle extends Command {

    private boolean isDone = false;
    private double current_angle = Robot.wristSubsystem.getWristPosition();
    private double targetArmAngle = Constants.Reef.coralEjectAngleLevel23;
    private int level = 0;

    public MoveToEjectCoralAngle(int level) {
        addRequirements(Robot.wristSubsystem);
        this.level = level;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (level == 1) {
            this.targetArmAngle = Constants.Reef.coralEjectAngleLevel1;
        } else if (level == 2 || level == 3) {
            this.targetArmAngle = Constants.Reef.coralEjectAngleLevel23;
        } else {
            this.targetArmAngle = Constants.Reef.coralEjectAngleLevel4;
        }
        isDone = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // set arm to correct angle
        current_angle = Robot.wristSubsystem.getWristPosition();
        // Calculate power curve proportional
        double armRotationPower = Math.abs(this.targetArmAngle - current_angle) / 80;
        // Move arm up or down to target arm angle
        if (Math.abs(this.targetArmAngle - current_angle) > Constants.Tolerances.coralEjectAngleTolerance) {
            double v_sign = Math.signum(this.targetArmAngle - current_angle);
            Robot.wristSubsystem.setWristSpeed(v_sign * (armRotationPower));
        } else {
            Robot.wristSubsystem.setWristSpeed(0d);
            //Robot.gamePieceManipulator.ejectCoral();
            isDone = true;
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

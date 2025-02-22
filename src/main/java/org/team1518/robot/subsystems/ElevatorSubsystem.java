// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1518.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1518.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    private TalonFX elevatorMotor;
    private double elevatorPosition = 0;
    private Encoder elevatorEncoder = new Encoder(0, 1);
    private boolean isMoving = false;

    public ElevatorSubsystem() {
        elevatorMotor = new TalonFX(Constants.Motors.elevatorMotorId);
        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void driveElevator(double speed) {
        elevatorPosition = getCurrentHeight();
        if (
            (speed > 0 && elevatorPosition < Constants.Limits.elevatorMax) ||
            (speed < 0 && elevatorPosition > Constants.Limits.elevatorMin)
        ) {
            elevatorMotor.set(speed);
            isMoving = true;
        } else {
            stopElevator();
        }
    }

    public void stopElevator() {
        elevatorMotor.set(0d);
        isMoving = false;
    }

    public double getCurrentHeight() {
        // get current height of the elevator
        elevatorPosition = -elevatorEncoder.get(); //elevatorMotor.getPosition().getValueAsDouble(); // this is in rotations
        return elevatorPosition / Constants.Factors.elevatorInchesPerRevolution;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Elevator Height", getCurrentHeight());
        SmartDashboard.putBoolean("Moving Elevator", isMoving);
    }
}

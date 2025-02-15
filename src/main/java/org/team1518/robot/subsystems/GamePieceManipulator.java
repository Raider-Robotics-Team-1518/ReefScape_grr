// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1518.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1518.robot.Constants;

public class GamePieceManipulator extends SubsystemBase {

    private final I2C.Port mxpI2cPort = I2C.Port.kMXP;
    private final I2C.Port builtInI2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_coralColorSensor = new ColorSensorV3(mxpI2cPort);
    private final ColorSensorV3 m_algaeColorSensor = new ColorSensorV3(builtInI2cPort);
    private TalonFX gamePieceMotor;

    public GamePieceManipulator() {
        gamePieceMotor = new TalonFX(Constants.Motors.gamePieceMotorId);
    }

    public boolean isCoralLoaded() {
        // read the color/presence sensor to see if the coral has been loaded
        boolean coralIsLoaded = m_coralColorSensor.getProximity() > 1024;
        SmartDashboard.putBoolean("Coral Is Loaded", coralIsLoaded);
        return coralIsLoaded;
    }

    public boolean isAlgaeLoaded() {
        // read the color/presence sensor to see if the coral has been loaded
        boolean algaeIsLoaded = m_algaeColorSensor.getProximity() > 1024;
        SmartDashboard.putBoolean("Algae Is Loaded", algaeIsLoaded);
        return algaeIsLoaded;
    }

    public void intakeCoral() {
        if (!isCoralLoaded()) {
            gamePieceMotor.set(Constants.MotorSpeeds.coralIntakeMotorSpeed);
        } else {
            stopGamePieceMotor();
        }
    }

    public void ejectCoral() {
        if (isCoralLoaded()) {
            gamePieceMotor.set(Constants.MotorSpeeds.coralEjectMotorSpeed);
        } else {
            stopGamePieceMotor();
        }
    }

    public void intakeAlgae() {
        if (!isAlgaeLoaded()) {
            gamePieceMotor.set(Constants.MotorSpeeds.algaeIntakeMotorSpeed);
        } else {
            stopAlgaeMotor();
        }
    }

    public void ejectAlgae() {
        if (isAlgaeLoaded()) {
            gamePieceMotor.set(Constants.MotorSpeeds.algaeEjectMotorSpeed);
        } else {
            stopAlgaeMotor();
        }
    }

    public void stopCoralMotor() {
        stopGamePieceMotor();
    }

    public void stopAlgaeMotor() {
        stopGamePieceMotor();
    }

    public void stopGamePieceMotor() {
        gamePieceMotor.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Is Coral Loaded", isCoralLoaded());
        SmartDashboard.putBoolean("Is Algae Loaded", isAlgaeLoaded());
    }
}

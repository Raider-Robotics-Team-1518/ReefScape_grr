// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1518.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1518.robot.Constants;

public class GamePieceManipulator extends SubsystemBase {

    private final I2C.Port mxpI2cPort = I2C.Port.kMXP;
    private final I2C.Port builtInI2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_coralColorSensor = new ColorSensorV3(builtInI2cPort);
    private final ColorSensorV3 m_algaeColorSensor = new ColorSensorV3(mxpI2cPort);
    private SparkMax gamePieceMotor = new SparkMax(Constants.Motors.gamePieceMotorId, MotorType.kBrushless);
    private SparkBaseConfig gamePieceMotorConfig;
    private final Rev2mDistanceSensor distanceSensor;
    private TalonFX gamePieceMotor2 = new TalonFX(Constants.Motors.gamePieceMotor2Id);

    public GamePieceManipulator() {
        gamePieceMotorConfig = new SparkMaxConfig();
        gamePieceMotorConfig.idleMode(IdleMode.kBrake);
        gamePieceMotorConfig.inverted(true);
        gamePieceMotor.configure(gamePieceMotorConfig, null, null);
        distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
        distanceSensor.setAutomaticMode(true);
        gamePieceMotor2.setNeutralMode(NeutralModeValue.Brake);
        gamePieceMotor2.setInverted(false);
    }

    public boolean isCoralLoaded() {
        // read the color/presence sensor to see if the coral has been loaded
        boolean coralIsLoaded = m_coralColorSensor.getProximity() > 1536;
        return coralIsLoaded;
    }

    public boolean isAlgaeLoaded() {
        // read the color/presence sensor to see if the coral has been loaded
        boolean algaeIsLoaded = m_algaeColorSensor.getProximity() > 100;
        return algaeIsLoaded;
    }

    public boolean isAtDistance() {
        return (distanceSensor.isRangeValid() && (distanceSensor.getRange() < 8));
    }

    public void intakeCoral() {
        gamePieceMotor.set(Constants.MotorSpeeds.coralIntakeMotorSpeed);
    }

    public void ejectCoral() {
        gamePieceMotor.set(Constants.MotorSpeeds.coralEjectMotorSpeed);
    }

    public void intakeAlgae() {
        gamePieceMotor.set(Constants.MotorSpeeds.algaeIntakeMotorSpeed);
        gamePieceMotor2.set(Constants.MotorSpeeds.algaeIntakeMotorSpeed);
    }

    public void ejectAlgae() {
        gamePieceMotor.set(Constants.MotorSpeeds.algaeEjectMotorSpeed);
        gamePieceMotor2.set(Constants.MotorSpeeds.algaeIntakeMotorSpeed);
    }

    public void runIntake(double speed) {
        gamePieceMotor.set(speed);
        gamePieceMotor2.set(speed);
    }

    public void stopCoralMotor() {
        stopGamePieceMotor();
    }

    public void stopAlgaeMotor() {
        stopGamePieceMotor();
    }

    public void stopGamePieceMotor() {
        gamePieceMotor.set(0);
        gamePieceMotor2.set(0);
    }

    public boolean isDistanceValid() {
        return distanceSensor.isRangeValid();
    }

    public double getDistanceToObject() {
        return distanceSensor.GetRange();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Is Coral Loaded", isCoralLoaded());
        SmartDashboard.putBoolean("Is Algae Loaded", isAlgaeLoaded());
        SmartDashboard.putBoolean("Is Within Range", isAtDistance());
        if (isDistanceValid()) {
            SmartDashboard.putNumber("Raw distance", distanceSensor.getRange());
        }
        SmartDashboard.putBoolean("Algae Sensor", m_algaeColorSensor.isConnected());
        SmartDashboard.putBoolean("Coral Sensor", m_coralColorSensor.isConnected());
    }
}

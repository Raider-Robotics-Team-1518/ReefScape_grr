package org.team1518.robot.subsystems;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.CANBus;
import com.studica.frc.AHRS;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.team1518.lib.swerve.Perspective;
import org.team1518.lib.swerve.SwerveAPI;
import org.team1518.lib.swerve.config.SwerveConfig;
import org.team1518.lib.swerve.config.SwerveModuleConfig;
import org.team1518.lib.swerve.hardware.SwerveEncoders;
import org.team1518.lib.swerve.hardware.SwerveIMUs;
import org.team1518.lib.swerve.hardware.SwerveMotors;
import org.team1518.lib.util.Tunable;
import org.team1518.lib.util.command.GRRSubsystem;
import org.team1518.robot.Constants;
import org.team1518.robot.Constants.RobotMap;

/**
 * The robot's swerve drivetrain.
 */
@Logged
public final class SwerveAuto extends GRRSubsystem {

    // Swerve Module Locations are based on the center of the robot.
    // +X is forward of center and +Y is left of center.
    // NavX IMU orientation is the USB port is directionally forward
    private static final SwerveModuleConfig kFrontLeft = new SwerveModuleConfig()
        .setName("frontLeft")
        .setLocation(0.3937, 0.3175)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.kFlMove, false))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.kFlTurn, true))
        .setEncoder(SwerveEncoders.canCoder(RobotMap.kFlEncoder, 0.260010, false));

    private static final SwerveModuleConfig kFrontRight = new SwerveModuleConfig()
        .setName("frontRight")
        .setLocation(0.3937, -0.3048)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.kFrMove, false))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.kFrTurn, true))
        .setEncoder(SwerveEncoders.canCoder(RobotMap.kFrEncoder, 0.129395, false));

    private static final SwerveModuleConfig kBackLeft = new SwerveModuleConfig()
        .setName("backLeft")
        .setLocation(-0.4699, 0.3175)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.kBlMove, false))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.kBlTurn, true))
        .setEncoder(SwerveEncoders.canCoder(RobotMap.kBlEncoder, 0.427002, false));

    private static final SwerveModuleConfig kBackRight = new SwerveModuleConfig()
        .setName("backRight")
        .setLocation(-0.4699, -0.3048)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.kBrMove, false))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.kBrTurn, true))
        .setEncoder(SwerveEncoders.canCoder(RobotMap.kBrEncoder, 0.220703, false));

    private static final SwerveConfig kConfig = new SwerveConfig()
        .setTimings(TimedRobot.kDefaultPeriod, 0.004, 0.02)
        .setMovePID(0.01, 0.0, 0.0)
        .setMoveFF(0.05, 0.1)
        .setTurnPID(7.7, 0.0, 0.2)
        .setBrakeMode(true, true)
        .setLimits(5.0, 13.0, 7.0, 27.5)
        .setDriverProfile(4.5, 1.0, 0.15, 4.2, 2.0, 0.05)
        .setPowerProperties(Constants.kVoltage, 80.0, 60.0)
        .setMechanicalProperties(8.14, 12.8, 4.5, Units.inchesToMeters(4.0))
        .setOdometryStd(0.1, 0.1, 0.1)
        .setIMU(SwerveIMUs.navx(AHRS.NavXComType.kUSB1))
        .setPhoenixFeatures(new CANBus(RobotMap.kSwerveCANBus), false, false, false)
        .setModules(kFrontLeft, kFrontRight, kBackLeft, kBackRight);

    private static final double kAutoKp = 7.0;
    private static final double kAutoKi = 0.0;
    private static final double kAutoKd = 0.0;

    private static final double kAutoAngularKp = 5.0;
    private static final double kAutoAngularKi = 0.0;
    private static final double kAutoAngularKd = 0.0;

    private final SwerveAPI api;

    private final PIDController autoPIDx;
    private final PIDController autoPIDy;
    private final PIDController autoPIDangular;

    private Pose2d autoLast = null;
    private Pose2d autoNext = null;

    public SwerveAuto() {
        api = new SwerveAPI(kConfig);

        autoPIDx = new PIDController(kAutoKp, kAutoKi, kAutoKd);
        autoPIDy = new PIDController(kAutoKp, kAutoKi, kAutoKd);
        autoPIDangular = new PIDController(kAutoAngularKp, kAutoAngularKi, kAutoAngularKd);
        autoPIDangular.enableContinuousInput(-Math.PI, Math.PI);

        api.enableTunables("swerve/api");
        Tunable.pidController("swerve/autoPID", autoPIDx);
        Tunable.pidController("swerve/autoPID", autoPIDy);
        Tunable.pidController("swerve/autoPIDangular", autoPIDangular);
    }

    @Override
    public void periodic() {
        api.refresh();
    }

    /**
     * Returns the current blue origin relative pose of the robot.
     */
    @NotLogged
    public Pose2d getPose() {
        return api.state.pose;
    }

    /**
     * Tares the rotation of the robot. Useful for
     * fixing an out of sync or drifting IMU.
     */
    public Command tareRotation() {
        return commandBuilder("Swerve.tareRotation()")
            .onInitialize(() -> api.tareRotation(Perspective.kOperator))
            .isFinished(true)
            .ignoringDisable(true);
    }

    /**
     * Drives the robot using driver input.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     * @param angular The CCW+ angular speed to apply, from {@code [-1.0, 1.0]}.
     */
    public Command drive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier angular) {
        return commandBuilder("Swerve.drive()").onExecute(() ->
            api.applyDriverInput(
                x.getAsDouble(),
                y.getAsDouble(),
                angular.getAsDouble(),
                Perspective.kOperator,
                true,
                true
            )
        );
    }

    /**
     * Drives the modules to stop the robot from moving.
     * @param lock If the wheels should be driven to an X formation to stop the robot from being pushed.
     */
    public Command stop(boolean lock) {
        return commandBuilder("Swerve.stop(" + lock + ")").onExecute(() -> api.applyStop(lock));
    }

    /**
     * Stops the robot from moving, and cleans up auto-related telemetry.
     * This command should be ran at the end of an autonomous routine.
     */
    public Command finishAuto() {
        return commandBuilder("Swerve.finishAuto()")
            .onInitialize(() -> {
                autoLast = null;
                autoNext = autoLast;
            })
            .onExecute(() -> api.applyStop(false));
    }

    /**
     * Resets the pose of the robot, inherently seeding field-relative movement. This
     * method is not intended for use outside of creating an {@link AutoFactory}.
     * @param pose The new blue origin relative pose to apply to the pose estimator.
     */
    public void resetPose(Pose2d pose) {
        api.resetPose(pose);
    }

    /**
     * Follows a Choreo trajectory by moving towards the next sample. This method
     * is not intended for use outside of creating an {@link AutoFactory}.
     * @param sample The next trajectory sample.
     */
    public void followTrajectory(SwerveSample sample) {
        autoLast = autoNext;
        autoNext = sample.getPose();

        Pose2d pose = api.state.pose;
        api.applySpeeds(
            new ChassisSpeeds(
                sample.vx + autoPIDx.calculate(pose.getX(), sample.x),
                sample.vy + autoPIDy.calculate(pose.getY(), sample.y),
                sample.omega + autoPIDangular.calculate(pose.getRotation().getRadians(), sample.heading)
            ),
            Perspective.kBlue,
            true,
            false
        );
    }
}

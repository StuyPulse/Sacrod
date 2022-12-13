package com.stuypulse.robot.subsystems;

import java.util.Arrays;
import java.util.stream.Stream;

import com.kauailabs.navx.frc.AHRS;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve.Chassis;
import com.stuypulse.robot.subsystems.modules.SwerveModule;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.SPI;

public class Swerve extends SubsystemBase {
    
    /** MODULES **/
    private final SwerveModule[] modules;

    /** SENSORS **/
    private final AHRS gyro;

    /** ODOMETRY **/
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    private final Field2d field;
    private final FieldObject2d[] module2ds;


    public Swerve() {
        modules = new SwerveModule[] {
        };

        gyro = new AHRS(SPI.Port.kMXP);

        kinematics = new SwerveDriveKinematics(
                getModuleStream()
                        .map(x -> x.getLocation())
                        .toArray(Translation2d[]::new));
        odometry = new SwerveDriveOdometry(kinematics, getGyroAngle());

        field = new Field2d();
        module2ds = new FieldObject2d[modules.length];
        for (int i = 0; i < modules.length; ++i) {
            module2ds[i] = field.getObject(modules[i].getId()+"-2d");
        }

        reset(new Pose2d());
        SmartDashboard.putData("Field", field);
    }

    /** MODULE API **/

    public SwerveModule getModule(String id) {
        for (SwerveModule module : modules) {
            if (module.getId().equals(id))
                return module;
        }

        throw new IllegalArgumentException("Couldn't find module with ID \"" + id + "\"");
    }

    public SwerveModule[] getModules() {
        return modules;
    }

    public Stream<SwerveModule> getModuleStream() {
        return Arrays.stream(getModules());
    }

    public SwerveModuleState[] getModuleStates() {
        return getModuleStream().map(x -> x.getState()).toArray(SwerveModuleState[]::new);
    }

    public void reset(Pose2d pose) {
        odometry.resetPosition(pose, getGyroAngle());
    }

    /** MODULE STATES API **/

    private static double getSaturation(SwerveModuleState[] states) {
        double sat = 1;
        for (var state : states) {
            sat = Math.max(sat, state.speedMetersPerSecond / Chassis.MAX_SPEED);
        }
        return sat;
    }

    public void setStates(Vector2D velocity, double omega, boolean fieldRelative) {
        if (fieldRelative) {
            final Rotation2d correction = new Rotation2d(0.5 * omega * Settings.DT);

            ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(velocity.y,
                    -velocity.x, -omega,
                    getAngle().plus(correction));

            for (int i = 0; i < 8; ++i) {
                double saturation = getSaturation(kinematics.toSwerveModuleStates(speeds));

                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(velocity.y, -velocity.x,
                        -omega,
                        getAngle().plus(correction.times(1.0 / saturation)));
            }

            setStatesRetainAngle(speeds);
        } else {
            setStates(new ChassisSpeeds(velocity.y, -velocity.x, -omega));
        }
    }

    private void setStatesRetainAngle(ChassisSpeeds robotSpeed) {
        var moduleStates = kinematics.toSwerveModuleStates(robotSpeed);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Chassis.MAX_SPEED);
        for (int i = 0; i < modules.length; ++i) {
            var currentState = modules[i].getState();
            if (moduleStates[i].speedMetersPerSecond < 0.1) {
                modules[i].setTargetState(new SwerveModuleState(
                    moduleStates[i].speedMetersPerSecond, 
                    currentState.angle
                ));
            } else {
                modules[i].setTargetState(moduleStates[i]);
            }
        }
        // setStates(robotSpeed);
    }

    public void setStates(Vector2D velocity, double omega) {
        setStates(velocity, omega, true);
    }

    public void setStates(ChassisSpeeds robotSpeed) {
        setStates(kinematics.toSwerveModuleStates(robotSpeed));
    }

    public void setStates(SwerveModuleState... states) {
        if (states.length != modules.length) {
            throw new IllegalArgumentException(
                    "Number of desired module states does not match number of modules (" + modules.length + ")");
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Chassis.MAX_SPEED);

        for (int i = 0; i < states.length; ++i) {
            modules[i].setTargetState(states[i]);
        }
    }

    /** GYRO API */

    public Rotation2d getGyroAngle() {
        return gyro.getRotation2d();
    }

    /** ODOMETRY API */

    private void updateOdometry() {
        odometry.update(getGyroAngle(), getModuleStates());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public Rotation2d getAngle() {
        return getPose().getRotation();
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    // AngleVelocity anglevelocity = new AngleVelocity();

    @Override
    public void periodic() {
        updateOdometry();
        field.setRobotPose(getPose());

        var pose = getPose();
        for (int i = 0; i < modules.length; ++i) {
            module2ds[i].setPose(new Pose2d(
                pose.getTranslation().plus(modules[i].getLocation().rotateBy(getAngle())),
                modules[i].getState().angle.plus(getAngle())
            ));
        }

        // TODO: log angular velocity and velocity vector
        SmartDashboard.putNumber("Swerve/Pose X", getPose().getTranslation().getX());
        SmartDashboard.putNumber("Swerve/Pose Y", getPose().getTranslation().getY());
        SmartDashboard.putNumber("Swerve/Pose Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Gyro Angle", gyro.getRotation2d().getDegrees());
    }

    @Override
    public void simulationPeriodic() {
        // Integrate omega in simulation and store in gyro
        var speeds = getKinematics().toChassisSpeeds(getModuleStates());

        gyro.setAngleAdjustment(gyro.getAngle() - Math.toDegrees(speeds.omegaRadiansPerSecond * Settings.DT));
        // gyro.setAngleAdjustment(getPose().getRotation().getDegrees());
    }


}

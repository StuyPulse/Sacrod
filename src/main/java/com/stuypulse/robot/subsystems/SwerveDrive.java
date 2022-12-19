package com.stuypulse.robot.subsystems;

import java.util.Arrays;
import java.util.stream.Stream;

import com.kauailabs.navx.frc.AHRS;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import static com.stuypulse.robot.constants.Ports.Swerve.*;
import com.stuypulse.robot.constants.Settings.Swerve.FrontLeft;
import com.stuypulse.robot.constants.Settings.Swerve.FrontRight;
import com.stuypulse.robot.constants.Settings.Swerve.BackLeft;
import com.stuypulse.robot.constants.Settings.Swerve.BackRight;
import com.stuypulse.robot.constants.Settings.Swerve.Chassis;
import com.stuypulse.robot.subsystems.modules.SL_SimModule;
import com.stuypulse.robot.subsystems.modules.SL_SwerveModule;
import com.stuypulse.robot.subsystems.modules.SparkMax_Module;
import com.stuypulse.robot.subsystems.modules.SwerveModule;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;

public class SwerveDrive extends SubsystemBase {

    public static SwerveDrive getInstance() {
        if (RobotBase.isReal()) {
            // return new SwerveDrive(new SwerveModule[] {
            //     new SL_SwerveModule(FrontRight.ID, FrontRight.MODULE_OFFSET, Ports.Swerve.FrontRight.TURN, Ports.Swerve.FrontRight.ENCODER, FrontRight.ABSOLUTE_OFFSET, Ports.Swerve.FrontRight.DRIVE),
            //     new SL_SwerveModule(FrontLeft.ID, FrontLeft.MODULE_OFFSET, Ports.Swerve.FrontLeft.TURN, Ports.Swerve.FrontLeft.ENCODER, FrontLeft.ABSOLUTE_OFFSET, Ports.Swerve.FrontLeft.DRIVE),
            //     new SL_SwerveModule(BackLeft.ID, BackLeft.MODULE_OFFSET, Ports.Swerve.BackLeft.TURN, Ports.Swerve.BackLeft.ENCODER, BackLeft.ABSOLUTE_OFFSET, Ports.Swerve.BackLeft.DRIVE),
            //     new SL_SwerveModule(BackRight.ID, BackRight.MODULE_OFFSET, Ports.Swerve.BackRight.TURN, Ports.Swerve.BackRight.ENCODER, BackRight.ABSOLUTE_OFFSET, Ports.Swerve.BackRight.DRIVE)
            // });

            return new SwerveDrive(new SwerveModule[] {
                new SparkMax_Module(FrontRight.ID, FrontRight.MODULE_OFFSET, Ports.Swerve.FrontRight.TURN, Ports.Swerve.FrontRight.ENCODER, FrontRight.ABSOLUTE_OFFSET.getRotation2d(), Ports.Swerve.FrontRight.DRIVE),
                new SparkMax_Module(FrontLeft.ID, FrontLeft.MODULE_OFFSET, Ports.Swerve.FrontLeft.TURN, Ports.Swerve.FrontLeft.ENCODER, FrontLeft.ABSOLUTE_OFFSET.getRotation2d(), Ports.Swerve.FrontLeft.DRIVE),
                new SparkMax_Module(BackLeft.ID, BackLeft.MODULE_OFFSET, Ports.Swerve.BackLeft.TURN, Ports.Swerve.BackLeft.ENCODER, BackLeft.ABSOLUTE_OFFSET.getRotation2d(), Ports.Swerve.BackLeft.DRIVE),
                new SparkMax_Module(BackRight.ID, BackRight.MODULE_OFFSET, Ports.Swerve.BackRight.TURN, Ports.Swerve.BackRight.ENCODER, BackRight.ABSOLUTE_OFFSET.getRotation2d(), Ports.Swerve.BackRight.DRIVE)
            });
        } else {
            return new SwerveDrive(new SwerveModule[] {
                new SL_SimModule(FrontRight.ID, FrontRight.MODULE_OFFSET),
                new SL_SimModule(FrontLeft.ID, FrontLeft.MODULE_OFFSET),
                new SL_SimModule(BackLeft.ID, BackLeft.MODULE_OFFSET),
                new SL_SimModule(BackRight.ID, BackRight.MODULE_OFFSET)
            });
        }
    }


    /** MODULES **/
    private final SwerveModule[] modules;

    /** SENSORS **/
    private final AHRS gyro;

    /** ODOMETRY **/
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    private final Field2d field;
    private final FieldObject2d[] module2ds;


    public SwerveDrive(SwerveModule[] modules) {
        this.modules = modules;

        gyro = new AHRS(SPI.Port.kMXP);

        kinematics = new SwerveDriveKinematics(
                getModuleStream()
                        .map(x -> x.getLocation())
                        .toArray(Translation2d[]::new));
        odometry = new SwerveDriveOdometry(kinematics, getGyroAngle(), getModulePositions());

        field = new Field2d();
        module2ds = new FieldObject2d[modules.length];
        for (int i = 0; i < modules.length; ++i) {
            module2ds[i] = field.getObject(modules[i].getId()+"-2d");
        }

        reset(new Pose2d());
        SmartDashboard.putData("Field", field);
    }

    public Field2d getField() {
        return field;
    }

    /** MODULE API **/

    public SwerveModule getModule(String id) {
        for (SwerveModule module : modules) {
            if (module.getId().equals(id))
                return module;
        }

        throw new IllegalArgumentException("Couldn't find module with ID \"" + id + "\"");
    }

    public ChassisSpeeds getChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getModuleStates());
    }

    public Translation2d getVelocity() {
        var speeds = getChassisSpeeds();
        return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    public Translation2d getTranslation() {
        return getPose().getTranslation();
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

    public SwerveModulePosition[] getModulePositions() {
        return getModuleStream().map(x -> x.getModulePosition()).toArray(SwerveModulePosition[]::new);
    }

    public void reset(Pose2d pose) {
        odometry.resetPosition(getGyroAngle(), getModulePositions(), pose);
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

            Pose2d next = new Pose2d(
                speeds.vxMetersPerSecond * Settings.DT, 
                speeds.vyMetersPerSecond * Settings.DT, 
                new Rotation2d(speeds.omegaRadiansPerSecond * Settings.DT));

            Twist2d twist = new Pose2d().log(next);

            speeds = new ChassisSpeeds(twist.dx/Settings.DT, twist.dy/Settings.DT, twist.dtheta/Settings.DT);

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
            if (Math.abs(moduleStates[i].speedMetersPerSecond) < Settings.Swerve.MIN_MODULE_VELOCITY) {
                modules[i].setTargetState(new SwerveModuleState(
                    moduleStates[i].speedMetersPerSecond, 
                    currentState.angle
                ));
            } else {
                modules[i].setTargetState(moduleStates[i]);
            }
        }
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
        odometry.update(getGyroAngle(), getModulePositions());
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

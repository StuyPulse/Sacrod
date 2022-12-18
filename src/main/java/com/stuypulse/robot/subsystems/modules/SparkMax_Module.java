package com.stuypulse.robot.subsystems.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Encoder;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 * 
 */
public class SparkMax_Module extends SwerveModule {

    /** MODULE **/

    private final String id;
    private final Translation2d location;

    private SwerveModuleState targetState;
    private double lastTargetSpeed;

    /** DRIVING **/

    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;

    private final SparkMaxPIDController drivePID;
    private final SimpleMotorFeedforward driveFF;

    /** TURNING **/

    private final CANSparkMax turnMotor;
    private final RelativeEncoder turnEncoder;
    private final DutyCycleEncoder turnAbsoluteEncoder;
    private final Rotation2d turnAbsoluteOffset;

    private final SparkMaxPIDController turnPID;

    public SparkMax_Module(String id, Translation2d location, int turnId, int encoderPort, Rotation2d absoluteOffset, int driveId) {
        // Module
        this.id = id;
        this.location = location;

        targetState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        lastTargetSpeed = 0.0;

        // Driving
        driveMotor = new CANSparkMax(driveId, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();

        drivePID = driveMotor.getPIDController();
        driveFF = new SimpleMotorFeedforward(Drive.kS.get(), Drive.kV.get(), Drive.kA.get());

        // Turning
        turnMotor = new CANSparkMax(turnId, MotorType.kBrushless);
        turnEncoder = turnMotor.getEncoder();

        turnAbsoluteEncoder = new DutyCycleEncoder(encoderPort);
        turnAbsoluteOffset = absoluteOffset;

        turnPID = turnMotor.getPIDController();

        // Configuration
        configure();
    }

    /** CONFIGURATION **/

    private void seedTurnEncoder() {
        turnEncoder.setPosition(getAbsoluteRadians());
    }

    private void configure() {
        // Configure drive motors, sensors

        driveMotor.restoreFactoryDefaults();

        driveMotor.enableVoltageCompensation(12.0);

        driveEncoder.setPositionConversionFactor(Encoder.Drive.POSITION_CONVERSION);
        driveEncoder.setVelocityConversionFactor(Encoder.Drive.VELOCITY_CONVERSION);
        driveEncoder.setPosition(0.0);

        drivePID.setP(Drive.kP.get()/12.0);
        drivePID.setI(Drive.kI.get());
        drivePID.setD(Drive.kD.get());
        drivePID.setDFilter(1.0);

        Motors.Swerve.Drive.configure(driveMotor); // Do this last because burnFlash

        // Configure turn motors, sensors

        turnMotor.restoreFactoryDefaults();

        turnMotor.enableVoltageCompensation(12.0);

        turnEncoder.setPositionConversionFactor(Encoder.Turn.POSITION_CONVERSION);
        turnEncoder.setVelocityConversionFactor(Encoder.Turn.VELOCITY_CONVERSION);
        seedTurnEncoder();

        turnPID.setP(Turn.kP.get());
        turnPID.setI(Turn.kI.get());
        turnPID.setD(Turn.kD.get());

        turnPID.setDFilter(1.0);

        Motors.Swerve.Turn.configure(turnMotor);
    }

    /** MODULE METHODS **/

    public String getId() {
        return id;
    }

    public Translation2d getLocation() {
        return location;
    }

    public void setTargetState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, getRotation());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), getRotation());
    }

    /** DRIVE METHODS **/

    /** reads velocity from the built-in encoder */
    public double getSpeed() {
        return driveEncoder.getVelocity();
    }

    /** TURN METHODS **/

    /**
     * reads the non-zeroed absolute encoder value (forward may not be zero if this
     * is called)
     */
    private double getRawAbsoluteRadians() {
        return MathUtil.interpolate(-Math.PI, +Math.PI, turnAbsoluteEncoder.getAbsolutePosition());
    }

    /** reads radians directly from the absolute encoder */
    public double getAbsoluteRadians() {
        return getRawAbsoluteRadians() - turnAbsoluteOffset.getRadians();
    }

    /**
     * stores the angle from the absolute encoder in a Rotation2d
     * so that operations on angles may have their value normalized
     * in the range (-Math.PI, +Math.PI)
     */
    public Rotation2d getAbsoluteRotation() {
        return new Rotation2d(getAbsoluteRadians());
    }

    /** reads radians directly from the built-in encoder */
    public double getRadians() {
        return turnEncoder.getPosition();
    }

    /**
     * stores the angle from the build-in encoder in a Rotation2d so
     * that operations on angles may have their value normalized in the
     * range (-Math.PI, +Math.PI)
     */
    public Rotation2d getRotation() {
        return new Rotation2d(getRadians());
    }

    @Override
    public void periodic() {
        // Update drive control loop
        double ff = driveFF.calculate(lastTargetSpeed, targetState.speedMetersPerSecond, Settings.DT);
        drivePID.setReference(lastTargetSpeed, ControlType.kVelocity, 0, ff, ArbFFUnits.kVoltage);
        lastTargetSpeed = targetState.speedMetersPerSecond;

        // Update angle control loop
        Rotation2d error = targetState.angle.minus(getRotation());

        // read directly from encoder here so that when error is calculated on the
        // SparkMax it is
        // cancelled out, and the leftover error is the shortest path (handled by using
        // Rotation2d's)
        // to calculate error
        turnPID.setReference(getRadians() + error.getRadians(), ControlType.kPosition);
   
        // Network Logging
        SmartDashboard.putNumber("Swerve/" + id + "/Velocity", getSpeed());
        SmartDashboard.putNumber("Swerve/" + id + "/Target Velocity", targetState.speedMetersPerSecond);

        
        SmartDashboard.putNumber("Swerve/" + id + "/Angle",
                MathUtil.inputModulus(getRotation().getDegrees(), -180, +180));
        SmartDashboard.putNumber("Swerve/" + id + "/Target Angle",
                MathUtil.inputModulus(targetState.angle.getDegrees(), -180, +180));

        SmartDashboard.putNumber("Swerve/" + id + "/Raw Absolute Angle", Math.toDegrees(getRawAbsoluteRadians()));
        SmartDashboard.putNumber("Swerve/" + id + "/Absolute Angle", Math.toDegrees(getAbsoluteRadians()));
    }
   
    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getRotation());
    }
   
    @Override
    public void reset() {
        setTargetState(new SwerveModuleState());
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(0);
    }
}

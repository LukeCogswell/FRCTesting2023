// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.SwerveModuleConstants.*;
import static frc.robot.Constants.SwerveModuleConstants.PID.*;
import static frc.robot.Constants.MeasurementConstants.*;
import static frc.robot.Constants.CANConstants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  
  private SwerveModulePosition m_modulePosition;
  
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_steerMotor;

  private final RelativeEncoder m_driveRelativeEncoder;
  private final RelativeEncoder m_steerRelativeEncoder;
  private final CANCoder m_steerEncoder;

  private final PIDController m_steerPIDController;

  private final double m_steerEncoderOffset;

  /**
   * Swerve Module class that contains the drive and steer motors, along with their encoders
   * <p>
   * IDs found in {@link CANConstants}
   * 
   * @param driveMotorID - CAN ID of the drive motor
   * @param steerMotorID - CAN ID of the steer motor
   * @param steerEncoderID - CAN ID of the encoder
   * @param steerEncoderOffset
   */
  public SwerveModule(int driveMotorID, int steerMotorID, int steerEncoderID, double steerEncoderOffset) {
    m_steerEncoderOffset = steerEncoderOffset;
    
    m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    m_steerMotor = new CANSparkMax(steerMotorID, MotorType.kBrushless);
    
    m_driveRelativeEncoder = m_driveMotor.getEncoder();
    m_steerRelativeEncoder = m_steerMotor.getEncoder();
    m_steerEncoder = new CANCoder(steerEncoderID);

    setMotorSettings(m_driveMotor, kDriveMotorCurrentLimit);
    setMotorSettings(m_steerMotor, kSteerMotorCurrentLimit);

    m_driveRelativeEncoder.setPositionConversionFactor(kDriveEncoderPositionConversionFactor); // Gives meters
    m_driveRelativeEncoder.setVelocityConversionFactor(kDriveEncoderPositionConversionFactor / 60.0); // Gives meters per second

    m_steerRelativeEncoder.setPositionConversionFactor(kSteerEncoderPositionConversionFactor); // Gives degrees
    m_steerRelativeEncoder.setVelocityConversionFactor(kSteerEncoderPositionConversionFactor / 60.0); // Gives degrees per second
    recalibrateRelativeEncoder();

    m_modulePosition = new SwerveModulePosition();
  
    m_steerPIDController = new PIDController(
      kSteerP,
      kSteerI,
      kSteerD
    );
    m_steerPIDController.enableContinuousInput(0, 360);

    m_driveMotor.burnFlash();
    m_steerMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
  public void updatePosition(){
    m_modulePosition.angle = getSteerAngle();
    m_modulePosition.distanceMeters = getDriveDistance();
  }
  
  
  public SwerveModulePosition getPosition(){
    updatePosition();
    return m_modulePosition;
  }
  
  public Rotation2d getSteerAngle(){
    double angle = m_steerRelativeEncoder.getPosition() - m_steerEncoderOffset;
    return Rotation2d.fromDegrees(angle); 
  }

  public double getDriveDistance(){
    return m_driveRelativeEncoder.getPosition();
  }

  public SwerveModuleState getModuleState(){
    return new SwerveModuleState(m_driveMotor.getEncoder().getVelocity(), getSteerAngle());
  }

  public void recalibrateRelativeEncoder(){
    m_steerRelativeEncoder.setPosition(m_steerEncoder.getAbsolutePosition()); // sets relative encoder to absolute encoder's value
  }

  public void setDesiredState(SwerveModuleState state) {
    // System.out.println("Pre Optimize: " + state.speedMetersPerSecond);
    state = SwerveModuleState.optimize(state, getSteerAngle());
    // System.out.println("Post Optimize: " + state.speedMetersPerSecond);
    // System.out.println("Setting: " + (state.speedMetersPerSecond / kMaxSpeedMetersPerSecond));
    m_driveMotor.set(state.speedMetersPerSecond / kMaxSpeedMetersPerSecond);
    m_steerMotor.set(
      m_steerPIDController.calculate(
        m_steerRelativeEncoder.getPosition(), 
        state.angle.getDegrees() + m_steerEncoderOffset)
    );
  }
  
  private void setMotorSettings(CANSparkMax motor, int currentLimit) {
    motor.restoreFactoryDefaults(); // restores factory default settings in case something was changed
    // motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 45); // sets update rate of motor faults, applied output, and is follower value to 45 ms
    // motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20); // sets update rate of motor velocity, current, temperature, and voltage to 20ms
    // motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20); // sets update rate of motor position to every 20 ms
    motor.setIdleMode(CANSparkMax.IdleMode.kBrake); // sets idle mode to brake
    motor.enableVoltageCompensation(kMaxVoltage);
    motor.setSmartCurrentLimit(currentLimit);
    motor.setInverted(true);
  }
  
  public double getAbsoluteAngle() {
    return m_steerEncoder.getAbsolutePosition(); 
  }

  public void stop(){
    m_driveMotor.set(0);
    m_steerMotor.set(0);
  }
}

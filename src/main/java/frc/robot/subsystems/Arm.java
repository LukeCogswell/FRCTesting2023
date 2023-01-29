// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  public CANSparkMax elbowMotor = new CANSparkMax(9, MotorType.kBrushless);
  private CANSparkMax lShoulderMotor = new CANSparkMax(10, MotorType.kBrushless);
  private CANSparkMax rShoulderMotor = new CANSparkMax(11, MotorType.kBrushless);
  /** Creates a new Arm. */
  public Arm() {
    lShoulderMotor.setInverted(true);
    rShoulderMotor.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setElbowMotor(Double speed) {
    elbowMotor.set(speed);
  }

  public void setShoulderMotors(Double speed) {
    lShoulderMotor.set(speed);
    rShoulderMotor.set(speed);
  }

}

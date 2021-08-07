// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private CANSparkMax frontLeft, frontRight, backLeft, backRight;
  private DifferentialDrive diffDrive;
  private boolean driveInverted;
  private DifferentialDriveKinematics driveKinematics;
  private DifferentialDriveOdometry driveOdometry;
  
  /** Creates a new drivetrain. */
  public Drivetrain() {
    // motor instantiation
    frontLeft = new CANSparkMax(Constants.Drivetrain.fL_ID, MotorType.kBrushless);
    frontRight = new CANSparkMax(Constants.Drivetrain.fR_ID, MotorType.kBrushless);
    backLeft = new CANSparkMax(Constants.Drivetrain.bL_ID, MotorType.kBrushless);
    backRight = new CANSparkMax(Constants.Drivetrain.bR_ID, MotorType.kBrushless);


  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

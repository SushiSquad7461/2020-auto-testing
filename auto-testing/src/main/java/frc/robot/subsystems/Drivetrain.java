// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.kauailabs.navx.frc.AHRS;

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
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private CANSparkMax frontLeft, frontRight, backLeft, backRight;
  private boolean driveInverted;
  private DifferentialDriveKinematics driveKinematics;
  private DifferentialDriveOdometry driveOdometry;
  private AHRS nav;
  private SimpleMotorFeedforward leftFeedForward, rightFeedForward;
  private PIDController leftController, rightController;
  private CANEncoder leftEncoder, rightEncoder;

  public Drivetrain() {
    
    // motor instantiation
    frontLeft = new CANSparkMax(Constants.kDrivetrain.fL_ID, Constants.kDrivetrain.MOTOR_TYPE);
    frontRight = new CANSparkMax(Constants.kDrivetrain.fR_ID, Constants.kDrivetrain.MOTOR_TYPE);
    backLeft = new CANSparkMax(Constants.kDrivetrain.bL_ID, Constants.kDrivetrain.MOTOR_TYPE);
    backRight = new CANSparkMax(Constants.kDrivetrain.bR_ID, Constants.kDrivetrain.MOTOR_TYPE);

    // instantiate navX
    nav = new AHRS(SPI.Port.kMXP);
    nav.reset();

    // instantiate encoders
    leftEncoder = frontLeft.getEncoder();
    rightEncoder = frontRight.getEncoder();
    
    // create odometry
    driveKinematics = new DifferentialDriveKinematics(Constants.kDrivetrain.trackWidth);
    driveOdometry = new DifferentialDriveOdometry(getAngle());

    // create feedforward and pid controllers
    leftFeedForward = new SimpleMotorFeedforward(
      Constants.kDrivetrain.LEFT_kS,
      Constants.kDrivetrain.LEFT_kV);
    rightFeedForward = new SimpleMotorFeedforward(
      Constants.kDrivetrain.RIGHT_kS, 
      Constants.kDrivetrain.RIGHT_kV);

    leftController = new PIDController(
      Constants.kDrivetrain.LEFT_kP, 
      Constants.kDrivetrain.LEFT_kI, 
      Constants.kDrivetrain.LEFT_kD);
    rightController = new PIDController(
      Constants.kDrivetrain.RIGHT_kP,
      Constants.kDrivetrain.RIGHT_kI, 
      Constants.kDrivetrain.RIGHT_kD);

		// open loop inversion configuration
		frontLeft.setInverted(driveInverted);
		frontRight.setInverted(driveInverted);
		backLeft.follow(frontLeft, driveInverted);
		backRight.follow(frontRight, driveInverted);

    // more motor config
		frontLeft.setSmartCurrentLimit(Constants.kDrivetrain.CURRENT_LIMIT);
		frontRight.setSmartCurrentLimit(Constants.kDrivetrain.CURRENT_LIMIT);
		backLeft.setSmartCurrentLimit(Constants.kDrivetrain.CURRENT_LIMIT);
    backRight.setSmartCurrentLimit(Constants.kDrivetrain.CURRENT_LIMIT);
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  public void closedCurveDrive(double linearVelocity, double angularVelocity, boolean isQuickTurn) {
    ChassisSpeeds chassisSpeed;
    DifferentialDriveWheelSpeeds wheelSpeeds;
    double leftOutput, rightOutput;
    double leftFeedForwardOutput, rightFeedForwardOutput;

    double scaledLinearVel = isQuickTurn ?
      0 :
      linearVelocity * Constants.kDrivetrain.CONTROLLER_LINEAR_SCALING;
    double scaledAngularVel = angularVelocity * Constants.kDrivetrain.CONTROLLER_ANGULAR_SCALING;

    chassisSpeed = new ChassisSpeeds(scaledLinearVel, 0, scaledAngularVel);

    wheelSpeeds = driveKinematics.toWheelSpeeds(chassisSpeed);

    leftFeedForwardOutput = leftFeedForward.calculate(wheelSpeeds.leftMetersPerSecond);
    rightFeedForwardOutput = rightFeedForward.calculate(wheelSpeeds.rightMetersPerSecond);

    leftOutput = leftController.calculate(leftEncoder.getVelocity(), wheelSpeeds.leftMetersPerSecond);
    rightOutput = rightController.calculate(rightEncoder.getVelocity(), wheelSpeeds.rightMetersPerSecond);

    frontLeft.setVoltage(leftOutput + leftFeedForwardOutput);
    frontRight.setVoltage(rightOutput + rightFeedForwardOutput);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(nav.getAngle());
  }

  public double getHeading() {
    double heading = -nav.getYaw();
    if (heading > 180 || heading < 180) {
      heading = Math.IEEEremainder(heading, 360);
    }
    return heading;
  }

  public void zeroAngle() {
    nav.reset();
  }

  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public void resetOdometry() {
    driveOdometry.resetPosition(getPose(), getAngle());
    resetEncoders();
  }

  public void setOdometry(Trajectory traj) {
    driveOdometry.resetPosition(traj.getInitialPose(), getAngle());
  }

  public void updateOdometry() {
    driveOdometry.update(getAngle(), leftEncoder.getPosition(), rightEncoder.getPosition());
  }

  public DifferentialDriveOdometry getOdometry() {
    return driveOdometry;
  }

  public Pose2d getPose() {  
    return driveOdometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    frontLeft.setVoltage(leftVolts);
    frontRight.setVoltage(rightVolts);
  }

  public DifferentialDriveKinematics getKinematics() {
    return driveKinematics;
  }

  @Override
  public void simulationPeriodic() {
    
  }
}

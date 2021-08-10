// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.Encoder;
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
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private CANSparkMax frontLeft, frontRight, backLeft, backRight;
  private DifferentialDrive diffDrive;
  private boolean driveInverted;
  private DifferentialDriveKinematics driveKinematics;
  private DifferentialDriveOdometry driveOdometry;
  private AHRS nav;
  private SimpleMotorFeedforward leftFeedForward, rightFeedForward;
  private PIDController leftController, rightController;
  private CANEncoder leftEncoder, rightEncoder;
  private Encoder simEncoderLeft;
  private Encoder simEncoderRight;

  // sim
  public DifferentialDrivetrainSim driveSim;
  private EncoderSim leftEncoderSim;
  private EncoderSim rightEncoderSim;
  private SimDevice gyroSim;

  
  /** Creates a new drivetrain. */
  public Drivetrain() {
    // motor instantiation
    frontLeft = new CANSparkMax(Constants.kDrivetrain.fL_ID, MotorType.kBrushless);
    frontRight = new CANSparkMax(Constants.kDrivetrain.fR_ID, MotorType.kBrushless);
    backLeft = new CANSparkMax(Constants.kDrivetrain.bL_ID, MotorType.kBrushless);
    backRight = new CANSparkMax(Constants.kDrivetrain.bR_ID, MotorType.kBrushless);

    // instantiate navX
    nav = new AHRS(SPI.Port.kMXP);
    nav.reset();

    // instantiate encoders
    leftEncoder = frontLeft.getEncoder();
    rightEncoder = frontRight.getEncoder();
    
    // create the differential drive
    diffDrive = new DifferentialDrive(frontLeft, frontRight);
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

    // configure motor controllers
		backLeft.follow(frontLeft);
		backRight.follow(frontRight);

		// open loop inversion configuration
		frontLeft.setInverted(driveInverted);
		frontRight.setInverted(driveInverted);
		backLeft.setInverted(driveInverted);
		backRight.setInverted(driveInverted);

		// closed loop inversion configuration
		/*frontLeft.setInverted(driveInverted);
		frontRight.setInverted(!driveInverted);
		backLeft.setInverted(driveInverted);
		backRight.setInverted(!driveInverted);*/

		frontLeft.setSmartCurrentLimit(Constants.kDrivetrain.CURRENT_LIMIT);
		frontRight.setSmartCurrentLimit(Constants.kDrivetrain.CURRENT_LIMIT);
		backLeft.setSmartCurrentLimit(Constants.kDrivetrain.CURRENT_LIMIT);
    backRight.setSmartCurrentLimit(Constants.kDrivetrain.CURRENT_LIMIT);

    simEncoderLeft = new Encoder(0, 1);
    simEncoderRight = new Encoder(3,4);

    simEncoderLeft.setDistancePerPulse(Math.PI/7);
    simEncoderRight.setDistancePerPulse(Math.PI/7);
    
    // instantiate simulation objects
    driveSim = new DifferentialDrivetrainSim(
      DCMotor.getNEO(2),
      7.29,                      // gear reduction
      7,                         // moment of inertia of robot
      100,                       // weight of robot (kg)
      Units.inchesToMeters(3),   // radius of wheels
      0.59,                      // track width (m)
      null                       // standard deviations of encoders
    );
    leftEncoderSim = new EncoderSim(simEncoderLeft);
    rightEncoderSim = new EncoderSim(simEncoderRight);
    gyroSim = new SimDevice(SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]"));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
    //frontLeft.setVoltage(5);
    //frontRight.setVoltage(6);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(nav.getAngle());
  }

  public void zeroAngle() {
    nav.reset();
  }

  public void resetOdometry() {
    driveOdometry.resetPosition(getPose(), getAngle());
  }

  public void updateOdometry() {
    //driveOdometry.update(getAngle(), leftEncoder.getPosition(), rightEncoder.getPosition());
    driveOdometry.update(getAngle(), leftEncoderSim.getDistance(), rightEncoderSim.getDistance());
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
    // This method will be called once per scheduler run during simulation

    // set the imputs of the sim
    driveSim.setInputs(
      /*frontLeft.get() **/ frontLeft.getBusVoltage(),
      /*frontRight.get() **/ frontRight.getBusVoltage());

    // rate of updating the sim (s)
    driveSim.update(0.02);

    // update sensors
    leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
    leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
    rightEncoderSim.setDistance(driveSim.getRightVelocityMetersPerSecond());
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(5.0);
  }
}

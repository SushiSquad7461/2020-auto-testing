package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

import java.io.File;
import java.io.IOException;
import java.nio.file.FileSystem;
import java.nio.file.Path;
import java.util.List;

public class Ramsete {
    private TrajectoryConfig config;
    private DifferentialDriveVoltageConstraint voltageConstraint;
    private Drivetrain drive;
    private SimpleMotorFeedforward ramseteFF; 
    
    // path enums
    public enum Paths {
        FORWARD("output/forward.wpilib.json"),
        CIRCLE("output/circle.wpilib.json"),
        CURVE("output/curve.wpilib.json"),
        GOAL_INIT_TO_MID("output/goal init to mid.wpilib.json"),
        CENTER_INIT_TO_MID("output/center init to mid.wpilib.json"),
        DEPOT_INIT_TO_MID("output/depot init to mid.wpilib.json"),
        MID_TO_SCORING("output/mid to scoring.wpilib.json"),
        SCORING_TO_TRENCH("output/scoring to trench.wpilib.json"),
        TRENCH_TO_SCORING("output/trench to scoring.wpilib.json");
        private String json;
        Paths(String json) {
            this.json = json;
        }

        public Trajectory getTrajectory() {
            try {
                System.out.println(Filesystem.getDeployDirectory().toPath().resolve(json));
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(json);
                return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            } catch (IOException ex) {
                return null; 
            }
        }
    }

    public Ramsete(Drivetrain drive) {
        this.drive = drive;
        ramseteFF = new SimpleMotorFeedforward(
            Constants.kRamsete.kS,
            Constants.kRamsete.kV,
            Constants.kRamsete.kA);
        config = new TrajectoryConfig(
            Constants.kRamsete.MAX_METERS_PER_SECOND,
            Constants.kRamsete.MAX_ACCEL_METERS_PER_SECOND_SQUARED)
            .setKinematics(this.drive.getKinematics())
            .addConstraint(getVoltageConstraint());

        voltageConstraint = new DifferentialDriveVoltageConstraint(
            ramseteFF,
            this.drive.getKinematics(),
            Constants.kRamsete.MAX_VOLTAGE);
    }

    public SequentialCommandGroup createRamseteCommand(Paths path) {
        return new RamseteCommand(
            path.getTrajectory(),
            drive::getPose,
            new RamseteController(
                Constants.kRamsete.B,
                Constants.kRamsete.ZETA),
            ramseteFF,
            drive.getKinematics(),
            drive::getWheelSpeeds,
            new PIDController(
                Constants.kRamsete.LEFT_kP,
                Constants.kRamsete.LEFT_kI,
                Constants.kRamsete.LEFT_kD),
            new PIDController(
                Constants.kRamsete.RIGHT_kP,
                Constants.kRamsete.RIGHT_kI,
                Constants.kRamsete.RIGHT_kD),
            drive::tankDriveVolts,
            drive)
            .andThen(() -> drive.tankDriveVolts(0, 0));
    }

    public DifferentialDriveVoltageConstraint getVoltageConstraint() {
        return voltageConstraint;
    }

}

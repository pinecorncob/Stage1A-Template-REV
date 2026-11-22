/*
 * Copyright 2026 FRCSoftware
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
package first.robot;

import org.wpilib.framework.OpModeRobot;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.*;
import first.robot.simulation.DrivetrainSim;
import org.wpilib.drive.DifferentialDrive;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.hardware.imu.OnboardIMU.MountOrientation;

/**
 * The methods in this class are called automatically as described in the OpModeRobot documentation.
 * OpMode classes anywhere in the package (or sub-packages) where this class is located are
 * automatically registered to display in the Driver Station. If you change the name of this class
 * or the package after creating this project, you must also update the Main.java file in the
 * project.
 */
public class Robot extends OpModeRobot {

  // each motor controller must be set to have a unique deviceID
  private final SparkMax leftLeader = new SparkMax(CANPort.CAN_S0, 0, MotorType.kBrushless);
  private final SparkMax leftFollower = new SparkMax(CANPort.CAN_S0, 1, MotorType.kBrushless);
  private final SparkMax rightLeader = new SparkMax(CANPort.CAN_S0, 2, MotorType.kBrushless);
  private final SparkMax rightFollower = new SparkMax(CANPort.CAN_S0, 3, MotorType.kBrushless);

  // sets differential drive to send its values to the left and right leading motors'
  // setThrottle methods
  public final DifferentialDrive drivetrain_tank =
      new DifferentialDrive(leftLeader::setThrottle, rightLeader::setThrottle);

  // imu keeps track of the robot's 3d rotation (pitch, roll, yaw)
  // MountOrientation.FLAT tells the imu that the robot is starting out in a "flat" rotation
  // (parallel to ground)
  private final OnboardIMU imu = new OnboardIMU(MountOrientation.FLAT);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // configuring motors

    // configuration object for the SparkMax motor controller, which stores configuration settings
    // such as inverting
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.inverted(true);

    // configure left motors: leftFollower should be following leftLeader
    leftLeader.configure(
        leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFollower.configure(
        leftConfig.follow(leftLeader),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SparkMaxConfig rightConfig = new SparkMaxConfig();

    rightLeader.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(
        rightConfig.follow(rightLeader),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void simulationPeriodic() {
  }
}

/*
 * Copyright 2026 FRCSoftware
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
package first.robot.simulation;

import static org.wpilib.units.Units.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkMax;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.DifferentialDrivetrainSim;
import org.wpilib.simulation.OnboardIMUSim;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;

public class DrivetrainSim_Annotated {

  // to be clear: NEO is the name of the motor, and sparkmax is the name of the motor controller.
  // because one of the motors follows the other on each side of the drivetrain, we only 
  // have one sparkmax on each side

  // create sparkmax objects like you would on a real robot
  private final SparkMax leftSpark, rightSpark;

  // SparkMaxSim is a simulated representation of the a sparkmax and its attached motor(s)
  private final SparkMaxSim leftSparkSim, rightSparkSim;

  // simulates the encoder within the sparkmax
  private final SparkRelativeEncoderSim leftEncoderSim, rightEncoderSim;

  // real measurements of the robot (in this case a kitbot)
  private final double kGearRatio = 10.71;
  private final double kWheelRadiusMeters = 0.0762; // 3 inches
  private static final double kBusVoltage = 12.0;

  // wpilib provides an object called DifferentialDriveTrainSim that simulates the
  // state of the drivetrain
  // parameters:
    /*  driveMotor - A DCMotor representing the left side of the drivetrain.
        gearing - The gearing ratio between motor and wheel, as output over input. This must be the same ratio as the ratio used to identify or create the drivetrainPlant.
        j - The moment of inertia of the drivetrain about its center in kg-m².
        mass - The mass of the drivebase in kg.
        wheelRadius - The radius of the wheels on the drivetrain in meters.
        trackwidth - The robot's trackwidth, or distance between left and right wheels, in meters.
     */
  private final DifferentialDrivetrainSim m_driveSim =
      new DifferentialDrivetrainSim(
          DCMotor.getNEO(2), // 2 NEO motors on each side of the drivetrain. 
          kGearRatio,
          2.1, // MOI of 2.1 kg m^2 (from CAD model).
          26.5, // Mass of the robot is 26.5 kg.
          kWheelRadiusMeters, // Robot uses 3" radius (6" diameter) wheels.
          0.546, // Distance between wheels in meters.
          null);

  
  private final TelemetryTable table = Telemetry.getTable("Drivetrain");
  private final TelemetryTable leftMotorTable = table.getTable("LeftMotor");
  private final TelemetryTable rightMotorTable = table.getTable("RightMotor");

  // initializes all the variables
  public DrivetrainSim_Annotated(SparkMax leftSpark, SparkMax rightSpark) {
    this.leftSpark = leftSpark;
    this.rightSpark = rightSpark;

    this.leftSparkSim = new SparkMaxSim(leftSpark, DCMotor.getNEO(2));
    this.rightSparkSim = new SparkMaxSim(rightSpark, DCMotor.getNEO(2));

    this.leftEncoderSim = leftSparkSim.getRelativeEncoderSim();
    this.rightEncoderSim = rightSparkSim.getRelativeEncoderSim();

    // set initial position and rotation of the robot
    m_driveSim.setPose(new Pose2d(2.5, 2, Rotation2d.ZERO));

    // this has something to do with the fuelsim class
    FuelSim.robotPoseSupplier = m_driveSim::getPose;
  }

  // this method repeatedly updates the simulation
  // in Robot.java, this method is called in simulationPeriodic()
  public void periodic() {
    double leftMotorVoltage = leftSpark.getThrottle() * kBusVoltage;
    double rightMotorVoltage = rightSpark.getThrottle() * kBusVoltage;

    // setInputs tells the drivetrain sim to take in the voltages of the motors as inputs
    // to update with
    m_driveSim.setInputs(leftMotorVoltage, rightMotorVoltage);
    // advance the simulation 0.02 sec in time (to match how often simulationPeriodic() is called)
    m_driveSim.update(0.02);

    // update the simulated robot's imu with the robot's current rotation
    OnboardIMUSim.setYaw(m_driveSim.getHeading().getRadians());

    // setPosition updates the simulated encoders with the position
    // set position takes in the position in revolutions/rotations for the encoder
      // m_driveSim.getLeftPosition() -> distance left side of the robot has traveled
      // divide by kWheelRadiusMeters -> angular displacement of the wheel (rad)
      // * kGearRatio -> angular displacement of the motor which the encoder tracks
    leftEncoderSim.setPosition(
        Units.radiansToRotations(m_driveSim.getLeftPosition() / kWheelRadiusMeters * kGearRatio));
    leftEncoderSim.setVelocity(
        Units.radiansPerSecondToRotationsPerMinute(
            m_driveSim.getLeftVelocity() / kWheelRadiusMeters * kGearRatio));
    rightEncoderSim.setPosition(
        Units.radiansToRotations(m_driveSim.getRightPosition() / kWheelRadiusMeters * kGearRatio));
    rightEncoderSim.setVelocity(
        Units.radiansPerSecondToRotationsPerMinute(
            m_driveSim.getRightVelocity() / kWheelRadiusMeters * kGearRatio));

    table.log("Pose", m_driveSim.getPose(), Pose2d.struct);
    table.log("LeftPosition", Meters.of(m_driveSim.getLeftPosition()));
    table.log("RightPosition", Meters.of(m_driveSim.getRightPosition()));
    table.log("LeftVelocity", MetersPerSecond.of(m_driveSim.getLeftVelocity()));
    table.log("RightVelocity", MetersPerSecond.of(m_driveSim.getRightVelocity()));
  }

  // not sure what this is for
  public void setPose(Pose2d pose) {
    m_driveSim.setPose(pose);
  }
}

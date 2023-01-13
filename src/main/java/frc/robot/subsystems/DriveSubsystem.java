// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class DriveSubsystem extends SubsystemBase {
  
  public static double translationP, translationI, translationD, rotationP, rotationI, rotationD;


  public boolean isFieldOriented = true;


  private SwerveModule frontLeft = new SwerveModule(0, 1, true, true, "frontLeft");
  private SwerveModule frontRight = new SwerveModule(2, 3, true, true, "frontRight");
  private SwerveModule backLeft = new SwerveModule(4, 5, true, true, "backLeft");
  private SwerveModule backRight = new SwerveModule(6, 7, true, true, "backRight");
  


  private double kMaxSpeed = 10;
  private double kMaxAngularSpeed = 10;


  private Field2d  m_field = new Field2d();
  private SwerveDriveOdometry m_odometry;
  private SwerveDriveKinematics m_Kinematics;

  private SimDouble angle;
  private int dev;
  double desirtedRobotRot, simYaw = 0;


  Joysticks joyee;

  double xPositionForTesting = 0;

  SwerveModule[] swerveModules = {
    frontLeft, 
    frontRight,
    backLeft,
    backRight
  };

  SwerveModulePosition[] swerveModulePositions = {
    frontLeft.getPosition(),
    frontRight.getPosition(),
    backLeft.getPosition(),
    backRight.getPosition(),
    
  };


  AHRS navx = new AHRS(Port.kMXP);

  public DriveSubsystem() {
    joyee = new Joysticks();
    m_Kinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.kWheelBase / 2, -Constants.kTrackWidth / 2),
      new Translation2d(Constants.kWheelBase / 2, Constants.kTrackWidth / 2),
      new Translation2d(-Constants.kWheelBase / 2, -Constants.kTrackWidth / 2),
      new Translation2d(-Constants.kWheelBase / 2, Constants.kTrackWidth / 2)
    );

    dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    
    m_odometry = new SwerveDriveOdometry(m_Kinematics, getRotation2d(), swerveModulePositions);

    m_odometry.update(getRotation2d(), swerveModulePositions);
    
    putPIDValues();
    SmartDashboard.putData("Field", m_field);
  }


  @Override
  public void simulationPeriodic() {

    updatePIDValues();

    double x = -joyee.getX();
    double y = -joyee.getY();
    double rot = joyee.getRot();

    x = Math.abs(x) > 0.15 ? x : 0.0;
    y = Math.abs(y) > 0.15 ? y : 0.0;
    rot = Math.abs(rot) > 0.15 ? rot : 0.0;


    SmartDashboard.putNumber("TransStickX", x);
    SmartDashboard.putNumber("odometryX", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("odometryY", m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("odometryRot", m_odometry.getPoseMeters().getRotation().getDegrees());

    desirtedRobotRot += rot * 0.02; 

    this.setMotors(x, y, rot);


    swerveModulePositions[0] = frontLeft.getPosition();
    swerveModulePositions[1] = frontRight.getPosition();
    swerveModulePositions[2] = backLeft.getPosition();
    swerveModulePositions[3] = backRight.getPosition();

    SwerveModuleState[] moduleStates = {
      swerveModules[0].getState(),
      swerveModules[1].getState(),
      swerveModules[2].getState(),
      swerveModules[3].getState()
    };

    ChassisSpeeds chassisSpeed = m_Kinematics.toChassisSpeeds(moduleStates);
    double chassisRotationSpeed = chassisSpeed.omegaRadiansPerSecond;


    simYaw += chassisRotationSpeed * 0.02;

    
    angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(-Units.radiansToDegrees(simYaw));

    m_odometry.update(getRotation2d(), swerveModulePositions);
    
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  public void resetGyro(){
    navx.reset();
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public double getHeading(){
    return Math.IEEEremainder(-navx.getAngle(), 360); 
    //im pretty sure this is the same thing as doing module operator % 
  }

  public void setMotors(double x, double y, double rot){
    x *= -kMaxSpeed; y *= kMaxSpeed;
    rot *= kMaxAngularSpeed;

    ChassisSpeeds chassisSpeeds;

    if(isFieldOriented)
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, getRotation2d());
    else
      chassisSpeeds = new ChassisSpeeds(x, y, rot);

    

    SwerveModuleState[] moduleStates = m_Kinematics.toSwerveModuleStates(chassisSpeeds);
    this.setModuleStates(moduleStates);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxSpeed);


    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }


  public void putPIDValues(){
    SmartDashboard.putNumber("translationP", 0);
    SmartDashboard.putNumber("translationI", 0);
    SmartDashboard.putNumber("translationD", 0);
    SmartDashboard.putNumber("rotationP", 0);
    SmartDashboard.putNumber("rotationI", 0);
    SmartDashboard.putNumber("rotationD", 0);
}

  public void updatePIDValues(){
      translationP = SmartDashboard.getNumber("translationP", 0);
      translationI = SmartDashboard.getNumber("translationI", 0);
      translationD = SmartDashboard.getNumber("translationD", 0);
      rotationP = SmartDashboard.getNumber("rotationP", 0);
      rotationI = SmartDashboard.getNumber("rotationI", 0);
      rotationD = SmartDashboard.getNumber("rotationD", 0);
  }
  
}

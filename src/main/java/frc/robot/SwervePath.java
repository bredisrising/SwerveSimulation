package frc.robot;

import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

public class SwervePath {
    public List<State> trajectoryPath;
    public double beginning;
    public double end;
    public double deadZone;
    public int idx;

    public double prevTime;
    public SwervePath(double initHeading, double endHeading) {

        trajectoryPath = Robot.path.getStates();
        beginning = initHeading;
        end = endHeading;
        deadZone = 0.25;
        idx = 1;

     }
    public SwervePath(double initHeading, double endHeading, double deadzone) {
        trajectoryPath = Robot.path.getStates();
        end = endHeading;
        beginning = initHeading;
        deadZone = deadzone;
        idx = 1;
    }


    public Pose2d getPose(int idx, double heading) {
        Pose2d desiredPose = trajectoryPath.get(idx).poseMeters;


        // Calculating wanted heading
        double desiredHeading = (end-heading)/(trajectoryPath.size()-idx) + heading;

        return new Pose2d(desiredPose.getX(), desiredPose.getY(), new Rotation2d(Units.degreesToRadians(desiredHeading)));

    }

    public ChassisSpeeds getSpeeds(Pose2d currentPose) {
        Pose2d desiredPose = getPose(idx, currentPose.getRotation().getDegrees());        
        double desiredVelocity = trajectoryPath.get(idx).velocityMetersPerSecond;
        double deltaTime = trajectoryPath.get(idx).timeSeconds - trajectoryPath.get(idx-1).timeSeconds;

        Transform2d distance = desiredPose.minus(currentPose);
    
        if(Math.abs(distance.getX()) <= deadZone && Math.abs(distance.getY()) <= deadZone){
          if(idx < trajectoryPath.size()-1) {
            idx++;
            System.out.println(idx);
          } else {
            return ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, distance.getRotation().getRadians(), currentPose.getRotation());
          }
        }
        // Handy dandy trigonometry
        // Why u no work, me use mathematics
        /*double angle = Math.atan(distance.getX()/distance.getY());

        double xSpeed = desiredVelocity*Math.sin(angle);
        double ySpeed = desiredVelocity*Math.cos(angle);
        double rotation = distance.getRotation().getRadians();

        System.out.println(desiredVelocity);
        System.out.println(angle);
        System.out.println(xSpeed);
        System.out.println(ySpeed);*/
        
        double xSpeed = distance.getX();
        double ySpeed = distance.getY();
        double rotation = distance.getRotation().getRadians();

        xSpeed *= 2/deltaTime;
        ySpeed *= 2/deltaTime;
        rotation *= 0/deltaTime;
    
        ChassisSpeeds desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, currentPose.getRotation());

        return desiredSpeeds;
    }



    }

    




/*

*/

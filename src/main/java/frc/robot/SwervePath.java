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
    public SwervePath(double initHeading, double endHeading) {

        trajectoryPath = Robot.path.getStates();
        beginning = initHeading;
        end = endHeading;
        deadZone = 0.25;
        idx = 0;
    }
    public SwervePath(double initHeading, double endHeading, double deadzone) {
        trajectoryPath = Robot.path.getStates();
        end = endHeading;
        beginning = initHeading;
        deadZone = deadzone;
        idx = 0;
    }


    public Pose2d getPose(int idx, double heading) {
        Pose2d desiredPose = trajectoryPath.get(idx).poseMeters;

        // Calculating wanted heading
        double desiredHeading = (end-heading)/(trajectoryPath.size()-idx) + heading;

        return new Pose2d(desiredPose.getX(), desiredPose.getY(), new Rotation2d(Units.degreesToRadians(desiredHeading)));

    }

    public ChassisSpeeds getSpeeds(Pose2d currentPose) {
        Pose2d desiredPose = getPose(idx, currentPose.getRotation().getDegrees());        
    
        Transform2d distance = desiredPose.minus(currentPose);
    
        if(Math.abs(distance.getX()) <= deadZone && Math.abs(distance.getY()) <= deadZone){
          if(idx < trajectoryPath.size()-1) {
            idx++;
            System.out.println(idx);
          } else {
            return ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, distance.getRotation().getRadians(), currentPose.getRotation());
          }
        }
    
        double xSpeed = distance.getX();
        double ySpeed = distance.getY(); 
        double rotation = distance.getRotation().getRadians();
    
        xSpeed *= 1;
        ySpeed *= 1;
        rotation *= -0.5;
    
        ChassisSpeeds desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, currentPose.getRotation());

        return desiredSpeeds;
    }

    }

    




/*

*/

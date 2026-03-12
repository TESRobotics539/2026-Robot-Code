package frc.util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * A paired chassis speeds + module states representing a constrained swerve setpoint.
 *
 * <p>Adapted from frc5687/2023-robot (originally from Team 254).
 */
public class SwerveSetpoint {
    public ChassisSpeeds       chassisSpeeds;
    public SwerveModuleState[] moduleStates;

    public SwerveSetpoint(ChassisSpeeds chassisSpeeds, SwerveModuleState[] moduleStates) {
        this.chassisSpeeds = chassisSpeeds;
        this.moduleStates  = moduleStates;
    }
}

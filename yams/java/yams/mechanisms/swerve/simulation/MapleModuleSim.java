package yams.mechanisms.swerve.simulation;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import yams.mechanisms.config.SwerveModuleConfig;

/**
 * Class that wraps around {@link org.ironmaple.simulation.drivesims.SwerveModuleSimulation}
 */
public class MapleModuleSim {

  /**
   * MapleSim module.
   */
  public SelfControlledSwerveDriveSimulation.SelfControlledModuleSimulation mapleSimModule = null;

  /**
   * Configure the maple sim module
   *
   * @param simModule the {@link org.ironmaple.simulation.drivesims.SwerveModuleSimulation} object for
   *                                simulation
   * @param moduleConfig the {@link SwerveModuleConfig} to get the swerve characteristics and apply them to the simulated module.
   */
  public void configureSimModule(org.ironmaple.simulation.drivesims.SwerveModuleSimulation simModule,
                                 SwerveModuleConfig moduleConfig)
  {
    this.mapleSimModule = new SelfControlledSwerveDriveSimulation.SelfControlledModuleSimulation(simModule);
    if (moduleConfig.getDriveMotor().getConfig().getSupplyStallCurrentLimit().isPresent()
            && moduleConfig.getAzimuthMotor().getConfig().getSupplyStallCurrentLimit().isPresent()) {
        this.mapleSimModule.withCurrentLimits(
                Amps.of(moduleConfig.getDriveMotor().getConfig().getSupplyStallCurrentLimit().getAsInt()),
                Amps.of(moduleConfig.getAzimuthMotor().getConfig().getSupplyStallCurrentLimit().getAsInt()));
    } else {
        throw new IllegalArgumentException("MapleModuleSim couldn't find Current Limits!");
    }
  }

  /**
   * Update the position and state of the module.
   *
   * @param desiredState State the swerve module is set to.
   */
  public void updateStateAndPosition(SwerveModuleState desiredState)
  {
    mapleSimModule.runModuleState(desiredState);
  }

  /**
   * Runs a drive motor characterization on the sim module.
   *
   * @param desiredFacing the desired facing of the module
   * @param volts         the voltage to run
   */
  public void runDriveMotorCharacterization(Rotation2d desiredFacing, double volts)
  {
    mapleSimModule.runDriveMotorCharacterization(desiredFacing, volts);
  }

  /**
   * Runs a drive motor characterization on the sim module.
   *
   * @param volts the voltage to run
   */
  public void runAngleMotorCharacterization(double volts)
  {
    mapleSimModule.runSteerMotorCharacterization(volts);
  }

  /**
   * Get the simulated swerve module position.
   *
   * @return {@link SwerveModulePosition} of the simulated module.
   */
  public SwerveModulePosition getPosition()
  {
    return mapleSimModule.getModulePosition();
  }

  /**
   * Get the {@link SwerveModuleState} of the simulated module.
   *
   * @return {@link SwerveModuleState} of the simulated module.
   */
  public SwerveModuleState getState()
  {
    if (mapleSimModule == null)
    {
      return new SwerveModuleState();
    }
    SwerveModuleState state = mapleSimModule.getMeasuredState();
    state.angle = state.angle.minus(Rotation2d.kZero);
    return state;
  }
}

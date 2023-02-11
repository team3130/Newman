package frc.robot.Constants;

public class RotaryMotor extends RangeBasedMotor {
    public RotaryMotor() {
        placementArmGearboxRatio = 16d/61d;
        getPlacementArmGearInRatio = 12d/60d;
        ticksToRadiansPlacement = Constants.kEncoderResolution * 2 * Math.PI * placementArmGearboxRatio * getPlacementArmGearInRatio;
        radiansToTicksPlacement = 1/ticksToRadiansPlacement;
        placementArmP = 5.12295e-5 / 2;
        placementArmI = 0;
        placementArmD = 0;
        placementArmFDown = 0;
        placementArmFUp = 0;
        maxVelocityPlacementArm = Math.PI/4;
        maxAccelerationPlacementArm = Math.PI/8;
        sStrengthPlacementArm = 0;
        CAN_ID = Constants.CAN_RotaryArm;
    }
}
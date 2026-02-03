package frc.robot.subsystems;

public class SuperstructureConstants {
    public static enum TurretStates {
        AUTOAIM_FOR_HUB,
        AUTOAIM_FOR_PASSING,
        HUB_PREALIGNED_LOCATION,
        IDLE,
        MANUAL,
        POINT_DIRECTLY_BACK_FOR_PASSING;  
    }

    public static enum FuelManagementStates {
        IDLE_EXTENDED,
        IDLE_RETRACTED,
        INDEX_FUEL_EXTENDED,
        INDEX_FUEL_RETRACTED,
        INTAKE_FUEL,
        INTAKE_INDEX_FUEL;
    }
}

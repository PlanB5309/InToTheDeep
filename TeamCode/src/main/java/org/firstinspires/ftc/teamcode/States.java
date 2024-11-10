package org.firstinspires.ftc.teamcode;

public enum States {
    //Shared States
    START,
    PARK,
    DONE_FOR_NOW,

    //RightOdometery States
    DRIVE_TO_BAR_S,
    RAISE_TO_HIGH_BAR_S,
    LOWER_SPECIMEN_LIFT_S,
    OPEN_CLAWS_S,
    BACK_UP_FROM_SUBMERISBLE_S,
    DRIVE_TOWARDS_SAMPLE_S,
    LINE_UP_ON_SAMPLE_S,
    PICK_UP_SAMPLE_S,
    DRIVE_TO_SPECIMEN_S,
    DROP_SAMPLE_S,
    TURN_NEAR_SUBMERSIBLE_S,

    //LeftOdometery States
    RIDE_AT_DAWN_AWAY_FROM_WALL_S,
    DRIVE_CLOSE_TO_BASKET_S,
    AT_THE_BASKET_S,
    MOVE_ARM_1_S,
    MOVE_ARM_2_S,
    SPIT_OUT_KRILL_1_S,
    SPIT_OUT_KRILL_2_S,
    BACK_AWAY_FROM_BASKET_S,
    HEAD_TOWARDS_KRILL_L_S,
    EAT_KRILL_L_S,
    LOWER_ARM_S,


    //SpecimenClawStates
    NOT_RUNNING,
    LOADING,
    SCORING,
    CLAWS_UP,
    LIFT_DOWN,
    CLAWS_DOWN;
}

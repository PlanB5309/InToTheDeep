package org.firstinspires.ftc.teamcode;

public enum States {
    //Shared States
    START,
    PARK,
    DONE_FOR_NOW,

    //RightOdometery States
    DRIVE_TO_BAR_S,
    DRIVE_TO_BAR_JR_S,
    DRIVE_TO_BAR_THIRD_S,
    RAISE_TO_HIGH_BAR_S,
    LOWER_SPECIMEN_LIFT_S,
    OPEN_CLAWS_S,
    BACK_UP_FROM_SUBMERISBLE_S,
    DRIVE_TOWARDS_SAMPLE_S,
    LINE_UP_ON_SAMPLE_S,
    PICK_UP_SAMPLE_S,
    LINE_UP_ON_SPECIMEN_S,
    DRIVE_TO_SPECIMEN_S,
    DRIVE_TO_SPECIMEN_JR_S,
    DROP_SAMPLE_S,
    TURN_NEAR_SUBMERSIBLE_S,
    TURN_NEAR_SUBMERSIBLE_JR_S,
    TURN_NEAR_SUBMERSIBLE_THIRD_S,
    BACK_UP_FROM_WALL_S,
    BACK_UP_FROM_WALL_JR_S,

    //LeftOdometery States
    TO_BASKET_1_S,
    AT_BASKET_1_S,
    RAISE_EXTEND_ARM_S,
    SCORE_SAMPLE_1_S,
    RETRACT_ARM_1_S,
    AWAY_FROM_BASKET_S,
    AT_BASKET_2_S,
    LINEUP_SAMPLE_1_S,
    LOAD_SAMPLE_1_S,
    TO_BASKET_2_S,
    LINEUP_SAMPLE_2_S,
    LINEUP_SAMPLE_3_S,
    LOAD_SAMPLE_2_S,
    LOAD_SAMPLE_3_S,
    TO_BASKET_3_S,
    TO_BASKET_4_S,



    //SpecimenClawStates
    NOT_RUNNING,
    LOADING,
    LOADING_JR,
    SCORING,
    SCORING_JR,
    SCORING_THIRD,
    CLAWS_UP,
    LIFT_DOWN,
    CLAWS_DOWN,
}

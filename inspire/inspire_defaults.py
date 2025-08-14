import inspire.inspire_dds as inspire_dds

#   name                       address  length  size
TOUCH_DATA = [
    ("fingerone_tip_touch",     3000,   18,     (3, 3)),      
    ("fingerone_top_touch",     3018,   192,    (12, 8)),      
    ("fingerone_palm_touch",    3210,   160,    (10, 8)),     
    ("fingertwo_tip_touch",     3370,   18,     (3, 3)),      
    ("fingertwo_top_touch",     3388,   192,    (12, 8)),      
    ("fingertwo_palm_touch",    3580,   160,    (10, 8)),     
    ("fingerthree_tip_touch",   3740,   18,     (3, 3)),    
    ("fingerthree_top_touch",   3758,   192,    (12, 8)),    
    ("fingerthree_palm_touch",  3950,   160,    (10, 8)),   
    ("fingerfour_tip_touch",    4110,   18,     (3, 3)),     
    ("fingerfour_top_touch",    4128,   192,    (12, 8)),     
    ("fingerfour_palm_touch",   4320,   160,    (10, 8)),    
    ("fingerfive_tip_touch",    4480,   18,     (3, 3)),     
    ("fingerfive_top_touch",    4498,   192,    (12, 8)),     
    ("fingerfive_middle_touch", 4690,   18,     (3, 3)),  
    ("fingerfive_palm_touch",   4708,   192,    (12, 8)),    
    ("palm_touch",              4900,   224,    (14, 8))                
]

def touch():
    return inspire_dds.inspire_hand_touch(
        fingerone_tip_touch      = [0] * 9,
        fingerone_top_touch      = [0] * 96,
        fingerone_palm_touch     = [0] * 80,
        fingertwo_tip_touch      = [0] * 9,
        fingertwo_top_touch      = [0] * 96,
        fingertwo_palm_touch     = [0] * 80,
        fingerthree_tip_touch    = [0] * 9,
        fingerthree_top_touch    = [0] * 96,
        fingerthree_palm_touch   = [0] * 80,
        fingerfour_tip_touch     = [0] * 9,
        fingerfour_top_touch     = [0] * 96,
        fingerfour_palm_touch    = [0] * 80,
        fingerfive_tip_touch     = [0] * 9,
        fingerfive_top_touch     = [0] * 96,
        fingerfive_middle_touch  = [0] * 9,
        fingerfive_palm_touch    = [0] * 96,
        palm_touch               = [0] * 112
    )


def state():
    return inspire_dds.inspire_hand_state(
        pos_act      = [0] * 6,
        angle_act    = [0] * 6,
        force_act    = [0] * 6,
        current      = [0] * 6,
        err          = [0] * 6,
        status       = [0] * 6,
        temperature  = [0] * 6
    )


def ctrl():
    return inspire_dds.inspire_hand_ctrl(
        pos_set     = [0] * 6,
        angle_set   = [0] * 6,
        force_set   = [0] * 6,
        speed_set   = [0] * 6,
        mode        = 0b0000
    )

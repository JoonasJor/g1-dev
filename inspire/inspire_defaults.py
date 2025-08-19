import inspire.inspire_dds as inspire_dds

#   name                       address  length  size
TOUCH_DATA = [
    ("little_tip_3",            3000,   18,     (3, 3)),      
    ("little_top_2",            3018,   192,    (12, 8)),      
    ("little_palm_1",           3210,   160,    (10, 8)),     

    ("ring_tip_3",              3370,   18,     (3, 3)),      
    ("ring_top_2",              3388,   192,    (12, 8)),      
    ("ring_palm_1",             3580,   160,    (10, 8)),     

    ("middle_tip_3",            3740,   18,     (3, 3)),    
    ("middle_top_2",            3758,   192,    (12, 8)),    
    ("middle_palm_1",           3950,   160,    (10, 8)),   

    ("index_tip_3",             4110,   18,     (3, 3)),     
    ("index_top_2",             4128,   192,    (12, 8)),     
    ("index_palm_1",            4320,   160,    (10, 8)),    

    ("thumb_tip_4",             4480,   18,     (3, 3)),     
    ("thumb_top_3",             4498,   192,    (12, 8)),     
    ("thumb_middle_2",          4690,   18,     (3, 3)),  
    ("thumb_palm_1",            4708,   192,    (12, 8)),    

    ("palm",                    4900,   224,    (14, 8))                
]

def touch():
    return inspire_dds.inspire_hand_touch(
        little_tip_3     = [0] * 9,
        little_top_2     = [0] * 96,
        little_palm_1    = [0] * 80,

        ring_tip_3       = [0] * 9,
        ring_top_2       = [0] * 96,
        ring_palm_1      = [0] * 80,

        middle_tip_3     = [0] * 9,
        middle_top_2     = [0] * 96,
        middle_palm_1    = [0] * 80,

        index_tip_3      = [0] * 9,
        index_top_2      = [0] * 96,
        index_palm_1     = [0] * 80,

        thumb_tip_4      = [0] * 9,
        thumb_top_3      = [0] * 96,
        thumb_middle_2   = [0] * 9,
        thumb_palm_1     = [0] * 96,

        palm             = [0] * 112
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

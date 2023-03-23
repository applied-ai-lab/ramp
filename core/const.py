import numpy as np

from core.utils.general_utils import AttrDict

BEAMS = {
    "b1": AttrDict(
        marker_ids=np.array([5, 6]),
        origin_id=5,
        offset=np.array([0, 0.0365, -0.0261]),
        joint_to_marker=AttrDict(
            j1=AttrDict(marker_id=5, offset=np.array([0, 0.0365, -0.0261])),
            j3=AttrDict(marker_id=6, offset=np.array([0, 0.0365, -0.0261])),
        ),
        joint_offsets=AttrDict(
            j1=np.array([0, 0.0, 0]),
            j2=np.array([0, -0.1462, 0]),
            j3=np.array([0, -0.2924, 0]),
        ),
        link_offsets=AttrDict(
            l1=np.array([0, -0.082125, 0]),
            l2=np.array([0.0, -0.210275, 0]),
        ),
    ),
    "b2": AttrDict(
        marker_ids=np.array([7, 8]),
        origin_id=7,
        offset=np.array([0, 0.0365, -0.0261]),
        joint_to_marker=AttrDict(
            j1=AttrDict(marker_id=7, offset=np.array([0, 0.0365, -0.0261])),
            j3=AttrDict(marker_id=8, offset=np.array([0, 0.0365, -0.0261])),
        ),
        joint_offsets=AttrDict(
            j1=np.array([0, 0.0, 0]),
            j2=np.array([0, -0.1462, 0]),
            j3=np.array([0, -0.2924, 0]),
        ),
        link_offsets=AttrDict(
            l1=np.array([0, -0.082125, 0]),
            l2=np.array([0.0, -0.210275, 0]),
        ),
    ),
    "b3": AttrDict(
        marker_ids=np.array([3, 4]),
        origin_id=3,
        offset=np.array([0, 0.0365, -0.0261]),
        joint_to_marker=AttrDict(
            j1=AttrDict(marker_id=3, offset=np.array([0, 0.0365, -0.0261])),
            j3=AttrDict(marker_id=4, offset=np.array([0, 0.0365, -0.0261])),
        ),
        joint_offsets=AttrDict(
            j1=np.array([0, 0.0, 0]),
            j2=np.array([0, -0.1462, 0]),
            j3=np.array([0, -0.2924, 0]),
        ),
        link_offsets=AttrDict(
            l1=np.array([0, -0.082125, 0]),
            l2=np.array([0.0, -0.210275, 0]),
        ),
    ),
    "b4": AttrDict(
        marker_ids=np.array([9, 10]),
        origin_id=9,
        offset=np.array([0, 0.0365, -0.0294]),
        joint_to_marker=AttrDict(
            j1=AttrDict(marker_id=9, offset=np.array([0, 0.0365, -0.0294])),
            j3=AttrDict(marker_id=10, offset=np.array([0, 0.0365, -0.0294])),
        ),
        joint_offsets=AttrDict(
            j1=np.array([0, 0.0, 0]),
            j2=np.array([0, -0.1462, 0]),
            j3=np.array([0, -0.2924, 0]),
        ),
        link_offsets=AttrDict(
            l1=np.array([0, -0.082125, 0]),
            l2=np.array([0.0, -0.210275, 0]),
        ),
    ),
    "b5": AttrDict(
        marker_ids=np.array([1, 2]),
        origin_id=1,
        offset=np.array([0, 0.0365, -0.0294]),
        joint_to_marker=AttrDict(
            j1=AttrDict(marker_id=1, offset=np.array([0, 0.0365, -0.0294])),
            j3=AttrDict(marker_id=2, offset=np.array([0, 0.0365, -0.0294])),
        ),
        joint_offsets=AttrDict(
            j1=np.array([0, -0.0, 0]),
            j2=np.array([0, -0.1462, 0]),
            j3=np.array([0, -0.2924, 0]),
        ),
        link_offsets=AttrDict(
            l1=np.array([0, -0.082125, 0]),
            l2=np.array([0.0, -0.210275, 0]),
        ),
    ),
    "b6": AttrDict(
        marker_ids=np.array([17, 18]),
        origin_id=17,
        offset=np.array([0, 0.0365, -0.0261]),
        joint_to_marker=AttrDict(
            j1=AttrDict(marker_id=17, offset=np.array([0, 0.0365, -0.0261])),
            j3=AttrDict(marker_id=18, offset=np.array([0, 0.0365, -0.0261])),
        ),
        joint_offsets=AttrDict(
            j1=np.array([0, 0.0, 0]),
            j2=np.array([0, 0.18952, 0]),
            j3=np.array([0, 0.37904, 0]),
        ),
        link_offsets=AttrDict(
            l1=np.array([0, 0.103785, 0]),
            l2=np.array([0.0, 0.275255, 0]),
        ),
    ),
    "b7": AttrDict(
        marker_ids=np.array([13, 14]),
        origin_id=13,
        offset=np.array([0, 0.0299, -0.0296]),
        joint_to_marker=AttrDict(
            j1=AttrDict(marker_id=13, offset=np.array([0, 0.0299, -0.0296])),
            j5=AttrDict(marker_id=14, offset=np.array([0, 0.0299, -0.0296])),
        ),
        joint_offsets=AttrDict(
            j1=np.array([0, 0.0, 0]),
            j2=np.array([0, -0.1262, 0]),
            j3=np.array([0, -0.2468, 0]),
            j4=np.array([0, -0.3674, 0]),
            j5=np.array([0, -0.4936, 0]),
        ),
        link_offsets=AttrDict(
            l1=np.array([0, -0.0582, 0]),
            l2=np.array([0, -0.1942, 0]),
            l3=np.array([0, -0.2994, 0]),
            l4=np.array([0, -0.4354, 0]),
        ),
    ),
    "b8": AttrDict(
        marker_ids=np.array([11, 12]),
        origin_id=11,
        offset=np.array([0, 0.0299, -0.0296]),
        joint_to_marker=AttrDict(
            j1=AttrDict(marker_id=11, offset=np.array([0, 0.0299, -0.0296])),
            j5=AttrDict(marker_id=12, offset=np.array([0, 0.0299, -0.0296])),
        ),
        joint_offsets=AttrDict(
            j1=np.array([0, 0.0, 0]),
            j2=np.array([0, -0.1262, 0]),
            j3=np.array([0, -0.2468, 0]),
            j4=np.array([0, -0.3674, 0]),
            j5=np.array([0, -0.4936, 0]),
        ),
        link_offsets=AttrDict(
            # l1=np.array([0, -0.0582, 0]),
            l1=np.array([0, -0.0482, 0]),
            # l2=np.array([0, -0.1942, 0]),
            l2=np.array([0, -0.1842, 0]),
            # l3=np.array([0, -0.2994, 0]),
            l3=np.array([0, -0.29, 0]),
            # l4=np.array([0, -0.4354, 0]),
            l4=np.array([0, -0.4254, 0]),
        ),
    ),
    "b9": AttrDict(
        marker_ids=np.array([15, 16]),
        origin_id=15,
        offset=np.array([0, 0.0365, -0.0261]),
        joint_to_marker=AttrDict(
            j1=AttrDict(marker_id=15, offset=np.array([0, 0.0365, -0.0261])),
            j5=AttrDict(marker_id=16, offset=np.array([0, 0.0365, -0.0261])),
        ),
        joint_offsets=AttrDict(
            j1=np.array([0, 0.0, 0]),
            j2=np.array([0, -0.1262, 0]),
            j3=np.array([0, -0.2468, 0]),
            j4=np.array([0, -0.3674, 0]),
            j5=np.array([0, -0.4936, 0]),
        ),
        link_offsets=AttrDict(
            l1=np.array([0, -0.062125, 0]),
            # l1=np.array([0, -0.072125, 0]),
            l2=np.array([0, -0.17888, 0]),
            l3=np.array([0, -0.3148, 0]),
            l4=np.array([0, -0.421475, 0]),
        ),
    ),
}

PEGS = AttrDict(p1=17, p2=18, p3=19, p4=21, p5=22, p6=23)

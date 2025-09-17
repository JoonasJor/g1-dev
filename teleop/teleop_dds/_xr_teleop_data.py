from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types

@dataclass
@annotate.final
@annotate.autoid("sequential")
class xr_teleop_data(idl.IdlStruct, typename="teleop.xr_teleop_data"):
    timestamp: types.float64
    data: types.sequence[types.uint8, 65535] # JSON payload
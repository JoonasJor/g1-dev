from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types

@dataclass
@annotate.final
@annotate.autoid("sequential")
class camera_image(idl.IdlStruct, typename="camera.camera_image"):
    timestamp: types.float64
    rgb: types.sequence[types.uint8, 65535]
    depth: types.sequence[types.uint8, 65535]
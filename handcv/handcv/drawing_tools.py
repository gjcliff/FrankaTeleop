import dataclasses
import enum
import math

import cv2
import numpy as np

# from mediapipe.framework.formats import landmark_pb2


@dataclasses.dataclass
class NormalizedLandmark:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    visibility: float = 1.0
    presence: float = 1.0

    def HasField(self, field_name: str = ""):
        return hasattr(self, field_name)


@dataclasses.dataclass
class NormalizedLandmarkList:
    landmark: list[NormalizedLandmark] = dataclasses.field(default_factory=list)


class HandLandmark(enum.IntEnum):
    """The 21 hand landmarks."""

    WRIST = 0
    THUMB_CMC = 1
    THUMB_MCP = 2
    THUMB_IP = 3
    THUMB_TIP = 4
    INDEX_FINGER_MCP = 5
    INDEX_FINGER_PIP = 6
    INDEX_FINGER_DIP = 7
    INDEX_FINGER_TIP = 8
    MIDDLE_FINGER_MCP = 9
    MIDDLE_FINGER_PIP = 10
    MIDDLE_FINGER_DIP = 11
    MIDDLE_FINGER_TIP = 12
    RING_FINGER_MCP = 13
    RING_FINGER_PIP = 14
    RING_FINGER_DIP = 15
    RING_FINGER_TIP = 16
    PINKY_MCP = 17
    PINKY_PIP = 18
    PINKY_DIP = 19
    PINKY_TIP = 20


HAND_PALM_CONNECTIONS = ((0, 1), (0, 5), (9, 13), (13, 17), (5, 9), (0, 17))

HAND_THUMB_CONNECTIONS = ((1, 2), (2, 3), (3, 4))

HAND_INDEX_FINGER_CONNECTIONS = ((5, 6), (6, 7), (7, 8))

HAND_MIDDLE_FINGER_CONNECTIONS = ((9, 10), (10, 11), (11, 12))

HAND_RING_FINGER_CONNECTIONS = ((13, 14), (14, 15), (15, 16))

HAND_PINKY_FINGER_CONNECTIONS = ((17, 18), (18, 19), (19, 20))

HAND_CONNECTIONS = frozenset().union(
    *[
        HAND_PALM_CONNECTIONS,
        HAND_THUMB_CONNECTIONS,
        HAND_INDEX_FINGER_CONNECTIONS,
        HAND_MIDDLE_FINGER_CONNECTIONS,
        HAND_RING_FINGER_CONNECTIONS,
        HAND_PINKY_FINGER_CONNECTIONS,
    ]
)


WHITE_COLOR = (224, 224, 224)
BLACK_COLOR = (0, 0, 0)
RED_COLOR = (0, 0, 255)
GREEN_COLOR = (0, 128, 0)
BLUE_COLOR = (255, 0, 0)


@dataclasses.dataclass
class DrawingSpec:
    # Color for drawing the annotation. Default to the white color.
    color: tuple[int, int, int] = WHITE_COLOR
    # Thickness for drawing the annotation. Default to 2 pixels.
    thickness: int = 2
    # Circle radius. Default to 2 pixels.
    circle_radius: int = 2


_RADIUS = 5
_RED = (48, 48, 255)
_GREEN = (48, 255, 48)
_BLUE = (192, 101, 21)
_YELLOW = (0, 204, 255)
_GRAY = (128, 128, 128)
_PURPLE = (128, 64, 128)
_PEACH = (180, 229, 255)
_WHITE = (224, 224, 224)
_CYAN = (192, 255, 48)
_MAGENTA = (192, 48, 255)

_PRESENCE_THRESHOLD = 0.5
_VISIBILITY_THRESHOLD = 0.5
_BGR_CHANNELS = 3

# Hands
_THICKNESS_WRIST_MCP = 3
_THICKNESS_FINGER = 2
_THICKNESS_DOT = -1

# Hand landmarks
_PALM_LANDMARKS = (
    HandLandmark.WRIST,
    HandLandmark.THUMB_CMC,
    HandLandmark.INDEX_FINGER_MCP,
    HandLandmark.MIDDLE_FINGER_MCP,
    HandLandmark.RING_FINGER_MCP,
    HandLandmark.PINKY_MCP,
)
_THUMP_LANDMARKS = (
    HandLandmark.THUMB_MCP,
    HandLandmark.THUMB_IP,
    HandLandmark.THUMB_TIP,
)
_INDEX_FINGER_LANDMARKS = (
    HandLandmark.INDEX_FINGER_PIP,
    HandLandmark.INDEX_FINGER_DIP,
    HandLandmark.INDEX_FINGER_TIP,
)
_MIDDLE_FINGER_LANDMARKS = (
    HandLandmark.MIDDLE_FINGER_PIP,
    HandLandmark.MIDDLE_FINGER_DIP,
    HandLandmark.MIDDLE_FINGER_TIP,
)
_RING_FINGER_LANDMARKS = (
    HandLandmark.RING_FINGER_PIP,
    HandLandmark.RING_FINGER_DIP,
    HandLandmark.RING_FINGER_TIP,
)
_PINKY_FINGER_LANDMARKS = (
    HandLandmark.PINKY_PIP,
    HandLandmark.PINKY_DIP,
    HandLandmark.PINKY_TIP,
)
_HAND_LANDMARK_STYLE = {
    _PALM_LANDMARKS: DrawingSpec(
        color=_RED, thickness=_THICKNESS_DOT, circle_radius=_RADIUS
    ),
    _THUMP_LANDMARKS: DrawingSpec(
        color=_PEACH, thickness=_THICKNESS_DOT, circle_radius=_RADIUS
    ),
    _INDEX_FINGER_LANDMARKS: DrawingSpec(
        color=_PURPLE, thickness=_THICKNESS_DOT, circle_radius=_RADIUS
    ),
    _MIDDLE_FINGER_LANDMARKS: DrawingSpec(
        color=_YELLOW, thickness=_THICKNESS_DOT, circle_radius=_RADIUS
    ),
    _RING_FINGER_LANDMARKS: DrawingSpec(
        color=_GREEN, thickness=_THICKNESS_DOT, circle_radius=_RADIUS
    ),
    _PINKY_FINGER_LANDMARKS: DrawingSpec(
        color=_BLUE, thickness=_THICKNESS_DOT, circle_radius=_RADIUS
    ),
}

_HAND_CONNECTION_STYLE = {
    HAND_PALM_CONNECTIONS: DrawingSpec(color=_GRAY, thickness=_THICKNESS_WRIST_MCP),
    HAND_THUMB_CONNECTIONS: DrawingSpec(color=_PEACH, thickness=_THICKNESS_FINGER),
    HAND_INDEX_FINGER_CONNECTIONS: DrawingSpec(
        color=_PURPLE, thickness=_THICKNESS_FINGER
    ),
    HAND_MIDDLE_FINGER_CONNECTIONS: DrawingSpec(
        color=_YELLOW, thickness=_THICKNESS_FINGER
    ),
    HAND_RING_FINGER_CONNECTIONS: DrawingSpec(
        color=_GREEN, thickness=_THICKNESS_FINGER
    ),
    HAND_PINKY_FINGER_CONNECTIONS: DrawingSpec(
        color=_BLUE, thickness=_THICKNESS_FINGER
    ),
}


def get_default_hand_landmarks_style() -> dict[int, DrawingSpec]:
    """Returns the default hand landmarks drawing style.

    Returns:
        A mapping from each hand landmark to its default drawing spec.
    """
    hand_landmark_style = {}
    for k, v in _HAND_LANDMARK_STYLE.items():
        for landmark in k:
            hand_landmark_style[landmark] = v
    return hand_landmark_style


def get_default_hand_connections_style() -> dict[tuple[int, int], DrawingSpec]:
    """Returns the default hand connections drawing style.

    Returns:
        A mapping from each hand connection to its default drawing spec.
    """
    hand_connection_style = {}
    for k, v in _HAND_CONNECTION_STYLE.items():
        for connection in k:
            hand_connection_style[connection] = v
    return hand_connection_style


def _normalized_to_pixel_coordinates(
    normalized_x: float,
    normalized_y: float,
    image_width: int,
    image_height: int,
) -> None | tuple[int, int]:
    """Converts normalized value pair to pixel coordinates."""

    # Checks if the float value is between 0 and 1.
    def is_valid_normalized_value(value: float) -> bool:
        return (value > 0 or math.isclose(0, value)) and (
            value < 1 or math.isclose(1, value)
        )

    if not (
        is_valid_normalized_value(normalized_x)
        and is_valid_normalized_value(normalized_y)
    ):
        # TODO: Draw coordinates even if it's outside of the image bounds.
        return None
    x_px = min(math.floor(normalized_x * image_width), image_width - 1)
    y_px = min(math.floor(normalized_y * image_height), image_height - 1)
    return x_px, y_px


def draw_landmarks(
    logger,
    image: np.ndarray,
    landmark_list: NormalizedLandmarkList,
    connections: list[tuple[int, int]] | None = None,
    landmark_drawing_spec: DrawingSpec | dict[int, DrawingSpec] = DrawingSpec(
        color=RED_COLOR
    ),
    connection_drawing_spec: DrawingSpec
    | dict[tuple[int, int], DrawingSpec] = DrawingSpec(),
    is_drawing_landmarks: bool = True,
):
    """Draws the landmarks and the connections on the image.

    Args:
      image: A three channel BGR image represented as numpy ndarray.
      landmark_list: A normalized landmark list proto message to be annotated
      on the image.
      connections: A list of landmark index tuples that specifies how landmarks
      to be connected in the drawing.
      landmark_drawing_spec: Either a DrawingSpec object or a mapping from hand
        landmarks to the DrawingSpecs that specifies the landmarks' drawing
        settings such as color, line thickness, and circle radius. If this
        argument is explicitly set to None, no landmarks will be drawn.
      connection_drawing_spec: Either a DrawingSpec object or a mapping from
      hand connections to the DrawingSpecs that specifies the connections'
      drawing settings such as color and line thickness. If this argument is
      explicitly set to None, no landmark connections will be drawn.
      is_drawing_landmarks: Whether to draw landmarks. If set false, skip
      drawing landmarks, only contours will be drawed.

    Raises:
      ValueError: If one of the followings:
        a) If the input image is not three channel BGR.
        b) If any connetions contain invalid landmark index.
    """
    if not landmark_list:
        logger.info("returning")
        return
    if image.shape[2] != _BGR_CHANNELS:
        raise ValueError("Input image must contain three channel bgr data.")
    image_rows, image_cols, _ = image.shape
    idx_to_coordinates = {}
    for idx, landmark in enumerate(landmark_list.landmark):
        if (
            landmark.HasField("visibility")
            and landmark.visibility < _VISIBILITY_THRESHOLD
        ) or (
            landmark.HasField("presence") and landmark.presence < _PRESENCE_THRESHOLD
        ):
            logger.info("continuing")
            logger.info(
                f"landmark.HasField('visibility'): {landmark.HasField('visibility')}"
            )
            logger.info(
                f"landmark.HasField('presence'): {landmark.HasField('presence')}"
            )
            continue
        landmark_px = _normalized_to_pixel_coordinates(
            landmark.x, landmark.y, image_cols, image_rows
        )
        if landmark_px:
            idx_to_coordinates[idx] = landmark_px
    if connections:
        num_landmarks = len(landmark_list.landmark)
        # Draws the connections if the start and end landmarks are both visible
        for connection in connections:
            start_idx = connection[0]
            end_idx = connection[1]
            if not (0 <= start_idx < num_landmarks and 0 <= end_idx < num_landmarks):
                raise ValueError(
                    f"Landmark index is out of range. Invalid connection "
                    f"from landmark #{start_idx} to landmark #{end_idx}."
                )
            if start_idx in idx_to_coordinates and end_idx in idx_to_coordinates:
                drawing_spec = (
                    connection_drawing_spec[connection]
                    if isinstance(connection_drawing_spec, dict)
                    else connection_drawing_spec
                )
                cv2.line(
                    image,
                    idx_to_coordinates[start_idx],
                    idx_to_coordinates[end_idx],
                    drawing_spec.color,
                    drawing_spec.thickness,
                )
    # Draws landmark points after finishing the connection lines, which is
    # aesthetically better.
    if is_drawing_landmarks and landmark_drawing_spec:
        for idx, landmark_px in idx_to_coordinates.items():
            drawing_spec = (
                landmark_drawing_spec[idx]
                if isinstance(landmark_drawing_spec, dict)
                else landmark_drawing_spec
            )
            # White circle border
            circle_border_radius = max(
                drawing_spec.circle_radius + 1,
                int(drawing_spec.circle_radius * 1.2),
            )
            cv2.circle(
                image,
                landmark_px,
                circle_border_radius,
                WHITE_COLOR,
                drawing_spec.thickness,
            )
            # Fill color into the circle
            cv2.circle(
                image,
                landmark_px,
                drawing_spec.circle_radius,
                drawing_spec.color,
                drawing_spec.thickness,
            )

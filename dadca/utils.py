"""
Math and geometry helper functions.
"""
import math
from typing import Tuple


def equilateral_third_point(
    A: Tuple[float, float],
    B: Tuple[float, float],
) -> Tuple[float, float]:
    """
    Returns the third vertex of an equilateral triangle with base A-B,
    offset perpendicularly to the side (90° CCW from A→B at the midpoint).

    Used for positioning the charging station.
    """
    mx, my = (A[0] + B[0]) / 2, (A[1] + B[1]) / 2
    dx, dy = B[0] - A[0], B[1] - A[1]
    length = math.sqrt(dx*dx + dy*dy)
    # Height of equilateral triangle = (√3/2) × side
    h = (math.sqrt(3) / 2) * length
    # Unit perpendicular (CCW rotation)
    px, py = -dy / length, dx / length
    return (mx + px * h, my + py * h)
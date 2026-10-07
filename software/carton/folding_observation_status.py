"""Exact schema for a fresh view with no usable carton identity.

This is an observation result, never permission to reuse a carton pose. A
consumer needs a separately registered, synchronized view before motion.
"""
from enum import Enum


class CartonAvailability(str, Enum):
    MISSING_IDENTITY = 'missing_carton_identity'


PARTIAL_VIEW_SCHEMA = 'carton_rgbd_missing_carton_identity/v1'
PARTIAL_VIEW_SOURCE = 'calibrated_rgbd_missing_carton_identity'

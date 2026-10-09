"""Assembly dimensions shared by the pilot twin, reach solver and carton frame.

273 mm shoulder-pan spacing comes from the XLeRobot 0.4.0 arm-base STEP plus
SO101 mounting-hole fit, approximately corroborated by the owner on 10 October
2026. See docs/xlerobot040-arm-spacing.md. It is not an angular calibration.
"""

ARM_SPACING_M = 0.273
SHOULDER_LEFT_M = {'left': ARM_SPACING_M / 2, 'right': -ARM_SPACING_M / 2}
SHOULDER_UP_M = 0.894  # Existing model height; not established by spacing measurement.
RIGHT_BASE_IN_MODEL_M = (-0.0388353, SHOULDER_LEFT_M['right'], SHOULDER_UP_M - 0.1166)

# The untouched upstream MJCF has 310.4 mm pan spacing. Its Base origins are
# 220 mm apart; each pan axis lies 45.2 mm farther outboard. Move only the two
# arm subtrees inward, retaining their midpoint, height, orientation and links.
VENDORED_ARM_SPACING_M = 0.3104
ARM_INWARD_SHIFT_M = (VENDORED_ARM_SPACING_M - ARM_SPACING_M) / 2

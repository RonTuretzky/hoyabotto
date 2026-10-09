"""Read-only carton identity/pixel observations for the LLM pilot.

Mapping: the 12-tag HACHIYO print kit (make_fold_box_tags.py / box_tag_geometry.py,
fold-box-tags checkout, 2026-10-09). Dimensions describe the print plan, not a
registration to the robot. This module deliberately does not solve a head pose.
"""
from __future__ import annotations

from farm.perception.gemma_tags import TagObserver, TagRobot, tool_schema

TOOL_NAME = "robot_get_carton_tags"
CARTON_ROLES = {
    10: "near_wall_center", 26: "near_wall_left", 27: "near_wall_right",
    21: "left_wall_far", 28: "left_wall_near", 22: "right_wall",
    25: "floor_center", 24: "floor_right",
    11: "left_short_flap", 12: "right_short_flap",
    13: "far_long_flap", 14: "near_long_flap",
}
MAPPING_SOURCE = "HACHIYO_12_tag_print_plan_2026-10-09_verify_physical_mounting"
LIMITS = (
    "Tag centres/corners are per-camera pixels, not pinch points or robot coordinates. "
    "Missing tags mean unknown/occluded, not a folded flap. Tags alone do not prove "
    "a grasp, fold, carton displacement or camera-body clearance. Head-to-arm "
    "registration is unverified; box-only pose translation must not drive the base."
)


class CartonTagRobot(TagRobot):
    tool_name = TOOL_NAME
    default_ids = tuple(CARTON_ROLES)

    def __init__(self, robot, *, clock=None):
        kwargs = {} if clock is None else {"clock": clock}
        super().__init__(robot, observer=TagObserver(
            roles=CARTON_ROLES, role_mapping_source=MAPPING_SOURCE, **kwargs))

    def schema(self, cameras):
        schema = tool_schema(cameras)
        f = schema["function"]
        f["name"] = TOOL_NAME
        f["description"] = (
            "Read the HACHIYO carton AprilTag36h11 kit from existing camera frames. "
            "Returns tag identities, pixel centres/corners, quality and frame provenance; "
            "no motor commands or camera revival. " + LIMITS)
        f["parameters"]["properties"]["tag_ids"]["description"] = (
            "Expected printed carton IDs; defaults to all 12: "
            + ", ".join(str(i) for i in self.default_ids))
        return schema


def carton_tag_lines(payload):
    """Compact supervisor text; keep frame provenance and all uncertainty explicit."""
    lines = ["Carton AprilTags: " + LIMITS,
             "Report only accepted IDs and copy their roles exactly. A panel with no accepted "
             "tag is unidentified, not necessarily invisible. Decision margin is a unitless score; "
             "shortest edge is measured in pixels. Keep the report to identified panels, "
             "right-short-flap identification and fold status; do not guess missing panels' roles."]
    if not isinstance(payload, dict):
        return lines + ["tags unavailable: malformed tool response"]
    result = payload.get("result") or {}
    observations = result.get("observations") or {}
    if not observations:
        return lines + ["tags unavailable: " + str(payload.get("error", "no observations"))[:200]]
    lines.append("Printed roles require mounting verification; wall/floor black squares 45 mm, "
                 "flap squares 35 mm, planned flap centres 90 mm above hinge (not measured).")
    for camera, row in observations.items():
        if row.get("status") == "UNKNOWN":
            lines.append(f"{camera}: UNKNOWN — {row.get('reason', 'unavailable')}")
            continue
        frame = row["frame"]
        lines.append(
            f"{camera}: {row['status']}; frame {frame['camera_id']} seq={frame['seq']} "
            f"stream={frame['stream_id']} sha256={frame['sha256']}; "
            f"captured_at={frame['captured_at']} received_at={frame['received_at']} "
            f"basis={frame['timestamp_basis']} age={frame['age_s_on_observer_clock']:.3f}s "
            f"new={frame['new_since_last_call']}; cross-host clock sync unverified; "
            f"image={row['image_size_px']} px (+x right, +y down)")
        for tag in row["tags"]:
            x, y = tag["center_px"]
            lines.append(f"  ID {tag['tag_id']} {tag['role']}: {tag['status']}, "
                         f"center=({x:.1f},{y:.1f}) px, edge={tag['shortest_edge_px']:.1f}px, "
                         f"margin_score={tag['decision_margin']:.1f} (unitless), hamming={tag['hamming']}")
        lines.append(f"  Missing/rejected expected IDs: {row['missing_ids']} (unknown, not absent/folded).")
    return lines

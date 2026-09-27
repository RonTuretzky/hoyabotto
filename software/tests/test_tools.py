"""bus_probe guess logic with a fake bus; light_monitor line formatting. No hardware."""
from farm.status import Reading, Status
from farm.tools import bus_probe, light_monitor


class FakeBus:
    def __init__(self, found, fail=None):
        self.found = found
        self.fail = fail
        self.disconnected = False

    def broadcast_ping(self, num_retry=0, raise_on_error=False):
        if self.fail:
            raise self.fail
        return self.found

    def disconnect(self, disable_torque=True):
        self.disconnected = True


def test_guess_bus():
    assert bus_probe.guess_bus(set(range(1, 9))) == "bus1 (left arm + head)"
    assert bus_probe.guess_bus({9, 10, 11, 12, 13, 14, 15, 16}) == "bus2 (right arm + wheels)"
    assert bus_probe.guess_bus({9, 10}) == "bus2 (right arm + wheels)"
    assert bus_probe.guess_bus(set(range(1, 8))) == "unknown"        # one head motor missing
    assert bus_probe.guess_bus(set()) == "unknown"


def test_probe_ports_with_fake_buses():
    buses = {
        "/dev/a": FakeBus({i: 777 for i in range(1, 9)}),
        "/dev/b": FakeBus({i: 777 for i in range(9, 17)}),
        "/dev/c": FakeBus(None),
        "/dev/d": FakeBus({}, fail=OSError("could not open port")),
    }

    def factory(port):
        if port == "/dev/e":
            raise FileNotFoundError(port)
        return buses[port]

    out = bus_probe.probe_ports(["/dev/a", "/dev/b", "/dev/c", "/dev/d", "/dev/e"], bus_factory=factory)
    by_port = {r["port"]: r for r in out}
    assert by_port["/dev/a"]["guess"] == "bus1 (left arm + head)" and by_port["/dev/a"]["ids"] == list(range(1, 9))
    assert by_port["/dev/a"]["models"][1] == "sts3215"
    assert by_port["/dev/b"]["guess"] == "bus2 (right arm + wheels)"
    assert "error" in by_port["/dev/c"] and "no response" in by_port["/dev/c"]["error"]
    assert by_port["/dev/d"]["error"].startswith("OSError")
    assert by_port["/dev/e"]["error"].startswith("FileNotFoundError")
    assert all(b.disconnected for p, b in buses.items() if p != "/dev/e")


def test_light_monitor_lines_carry_status():
    ok = Reading(412.5, Status.OK, seq=812, meta={"gap": False})
    gap = Reading(400.0, Status.STALE, seq=820, meta={"gap": True}, note="age 1.50s > 1.0s")
    bad = Reading(None, Status.INVALID, note="sensor error: i2c")
    assert "status=OK" in light_monitor.format_latest(ok) and "412.5" in light_monitor.format_latest(ok)
    assert "GAP" in light_monitor.format_latest(gap) and "STALE" in light_monitor.format_latest(gap)
    assert "---" in light_monitor.format_latest(bad) and "INVALID" in light_monitor.format_latest(bad)
    m = Reading({"median_lux": 400.0, "spread_lux": 3.0, "n": 10, "errors": 0, "saturated": False, "samples": []}, Status.OK)
    assert "median=400.0" in light_monitor.format_summary(m)
    assert "INVALID" in light_monitor.format_summary(Reading(None, Status.INVALID, note="only 2/10"))

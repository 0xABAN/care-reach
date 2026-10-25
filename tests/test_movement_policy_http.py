"""The movement mode route has a strict string API and keeps the boolean alias."""

from arm_control.movement_policy import parse_movement_mode
from test_single_motor_http import FakeArm, SingleMotorRouteTests


class ModeArm(FakeArm):
    def set_movement_mode(self, mode):
        selected = parse_movement_mode(mode)
        self.calls.append(selected.value)
        return {"movement_mode": selected.value}


class MovementModeRouteTests(SingleMotorRouteTests):
    def setUp(self):
        self.arm = ModeArm()
        self.handler_class = self.handler_for_arm()

    def handler_for_arm(self):
        from server import handler_for

        return handler_for(self.arm, 8765)

    def test_explicit_mode_values(self):
        for name in ("coordinated", "shoulder_isolated", "single_joint"):
            self.assertEqual(self.post({"movement_mode": name}),
                             (200, {"movement_mode": name}))
        self.assertEqual(self.arm.calls, ["coordinated", "shoulder_isolated", "single_joint"])

    def test_rejects_invalid_mode_payload(self):
        for body in ({"movement_mode": "other"}, {"movement_mode": True},
                     {"movement_mode": "shoulder_isolated", "single_motor_mode": False},
                     {"movement_mode": "coordinated", "extra": 1}):
            with self.subTest(body=body):
                status, _ = self.post(body)
                self.assertEqual(status, 400)
        self.assertEqual(self.arm.calls, [])

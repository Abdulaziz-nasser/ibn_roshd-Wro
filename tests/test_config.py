from robot.config import load_project_config


def test_project_configs_load():
    robot, vision = load_project_config("config/robot.yaml", "config/vision.yaml")
    assert robot["mission"]["max_turns"] >= 1
    assert all(name in vision["colors"] for name in ("BLUE", "ORANGE", "RED", "GREEN"))
    assert robot["drive"]["minimum_speed"] <= robot["drive"]["speed"]

from robot.protocol import parse_line, shortest_error, wrap180


def test_wrap180():
    assert wrap180(181) == -179
    assert wrap180(-181) == 179
    assert wrap180(540) == 180


def test_shortest_error():
    assert shortest_error(-170, 170) == 20
    assert shortest_error(170, -170) == -20


def test_telemetry_parser():
    parsed = parse_line(
        "TLM,boot=7,yaw=-12.5,state=DRIVE,steer=0.2,speed=34,dF=100,dL=20,dR=30,dB=40,enc_cm=12.3,armed=1"
    )
    telemetry = parsed.values["telemetry"]
    assert telemetry.boot_id == 7
    assert telemetry.yaw == -12.5
    assert telemetry.armed is True
    assert telemetry.enc_cm == 12.3


def test_done_parser():
    parsed = parse_line("DONE,id=42,result=STALLED,progress_cm=3.4,detail=no_encoder")
    result = parsed.values["result"]
    assert result.command_id == 42
    assert result.result == "STALLED"
    assert result.progress_cm == 3.4

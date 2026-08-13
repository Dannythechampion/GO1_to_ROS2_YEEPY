from types import SimpleNamespace


def test_shutdown_context_only_shuts_down_a_live_context():
    from omx_navigation.runtime_shutdown import shutdown_context

    events = []
    live = SimpleNamespace(ok=lambda: True, shutdown=lambda: events.append("shutdown"))
    stopped = SimpleNamespace(ok=lambda: False, shutdown=lambda: events.append("unexpected"))

    shutdown_context(live)
    shutdown_context(stopped)

    assert events == ["shutdown"]


def test_external_shutdown_is_a_normal_spin_exception():
    from omx_navigation.runtime_shutdown import (
        ExternalShutdownException,
        NORMAL_SHUTDOWN_EXCEPTIONS,
    )

    assert isinstance(ExternalShutdownException(), NORMAL_SHUTDOWN_EXCEPTIONS)
    assert isinstance(KeyboardInterrupt(), NORMAL_SHUTDOWN_EXCEPTIONS)

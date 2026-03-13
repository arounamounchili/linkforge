import logging

import pytest
from linkforge_core.logging_config import get_logger, setup_logging


@pytest.fixture(autouse=True)
def reset_logging():
    """Reset the linkforge logger between tests."""
    logger = logging.getLogger("linkforge")
    # Store original state
    original_handlers = logger.handlers[:]
    original_propagate = logger.propagate
    original_level = logger.level

    # Clear handlers for the test to allow caplog to work
    logger.handlers = []
    logger.propagate = True

    yield

    # Restore
    logger.handlers = original_handlers
    logger.propagate = original_propagate
    logger.setLevel(original_level)


def test_setup_logging_console(caplog) -> None:
    """Test logging setup with console only."""
    # setup_logging will add handlers to 'linkforge'
    setup_logging(console=True)
    logger = get_logger("test_console")

    # Force propagation for the test to ensure caplog (at root) sees it
    logging.getLogger("linkforge").propagate = True

    with caplog.at_level(logging.INFO, logger="linkforge"):
        logger.info("Test console message")

    assert "Test console message" in caplog.text
    assert logger.name == "linkforge.test_console"


def test_setup_logging_file(tmp_path) -> None:
    """Test logging setup with file."""
    log_file = tmp_path / "test.log"
    setup_logging(log_file=log_file, console=False)

    logger = get_logger("test_file")
    logger.info("Test file message")

    # Force flushing
    for handler in logging.getLogger("linkforge").handlers:
        handler.flush()
        if isinstance(handler, logging.FileHandler):
            handler.close()

    assert log_file.exists()
    content = log_file.read_text()
    assert "Test file message" in content


def test_get_logger() -> None:
    """Test that get_logger returns correctly named logger."""
    logger = get_logger("my_module")
    assert logger.name == "linkforge.my_module"

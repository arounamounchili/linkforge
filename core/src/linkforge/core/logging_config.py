"""Logging configuration for LinkForge.

This module provides centralized logging configuration for the LinkForge extension.
It supports both console and file logging with configurable log levels.
"""

from __future__ import annotations

import logging
from pathlib import Path

# Default log format
LOG_FORMAT = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
DATE_FORMAT = "%Y-%m-%d %H:%M:%S"


def setup_logging(
    log_file: Path | None = None,
    level: int = logging.INFO,
    console: bool = True,
) -> None:
    """Configure logging for LinkForge.

    Args:
        log_file: Optional path to log file
        level: Logging level (default: INFO)
        console: Whether to log to console (default: True)
    """
    handlers: list[logging.Handler] = []

    if console:
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(logging.Formatter(LOG_FORMAT, DATE_FORMAT))
        handlers.append(console_handler)

    if log_file:
        log_file.parent.mkdir(parents=True, exist_ok=True)
        file_handler = logging.FileHandler(log_file)
        file_handler.setFormatter(logging.Formatter(LOG_FORMAT, DATE_FORMAT))
        handlers.append(file_handler)

    # Configure root logger for LinkForge
    logger = logging.getLogger("linkforge")
    logger.setLevel(level)
    logger.handlers = handlers
    logger.propagate = False  # Don't propagate to root logger


def get_logger(name: str) -> logging.Logger:
    """Get a logger for a module.

    Args:
        name: Module name (usually __name__)

    Returns:
        Configured logger instance

    Example:
        >>> logger = get_logger(__name__)
        >>> logger.info("Starting export process")
    """
    return logging.getLogger(f"linkforge.{name}")

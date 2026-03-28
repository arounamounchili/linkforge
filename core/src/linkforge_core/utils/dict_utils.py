"""Dictionary utility functions for LinkForge."""

from __future__ import annotations

from typing import Any


class AttrDict(dict[str, Any]):
    """Dictionary providing attribute-access for nested fields (e.g., config.mass).
    Used for XACRO property resolution where YAML-loaded data uses dot notation.
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """Initialize the attribute dictionary and recursively wrap nested data.

        Args:
            *args: Positional arguments passed to the dict constructor.
            **kwargs: Keyword arguments passed to the dict constructor.
        """
        super().__init__(*args, **kwargs)
        for key, value in self.items():
            self[key] = self._wrap(value)

    @classmethod
    def _wrap(cls, value: Any) -> Any:
        """Recursively wrap dictionaries and lists.

        Args:
            value: The value to potentially wrap in an AttrDict.

        Returns:
            The wrapped value (AttrDict, list of wrapped values, or the original).
        """
        if isinstance(value, dict):
            return cls(value)
        if isinstance(value, list):
            return [cls._wrap(v) for v in value]
        return value

    def __getattr__(self, name: str) -> Any:
        """Access dictionary keys as attributes.

        Args:
            name: Key name to access.

        Returns:
            Value associated with the key.

        Raises:
            AttributeError: If key is not found.
        """
        try:
            return self[name]
        except KeyError:
            raise AttributeError(
                f"'{self.__class__.__name__}' object has no attribute '{name}'"
            ) from None

    def __setattr__(self, name: str, value: Any) -> None:
        """Set dictionary keys as attributes.

        Args:
            name: Key name to set.
            value: Value to associate with the key.
        """
        self[name] = value

    def __delattr__(self, name: str) -> None:
        """Delete dictionary keys as attributes.

        Args:
            name: Key name to delete.

        Raises:
            AttributeError: If key is not found.
        """
        try:
            del self[name]
        except KeyError:
            raise AttributeError(
                f"'{self.__class__.__name__}' object has no attribute '{name}'"
            ) from None

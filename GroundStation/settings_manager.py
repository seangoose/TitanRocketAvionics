# =============================================================================
#  TRES Titan Ground Station — settings_manager.py
#  Persists user-configurable session settings between launches.
#  Settings are stored in GroundStation/settings.json.
#  Atomic write (temp-file + rename) prevents corruption on power loss.
# =============================================================================

import json
import os
import tempfile

import config

# Path to the settings file, relative to the GroundStation directory.
# When main.py is run from GroundStation/ this resolves correctly.
_SETTINGS_FILE = os.path.join(os.path.dirname(__file__), "settings.json")

# Keys and their config.py fallback values
_DEFAULTS = {
    "lora_s1_freq":      config.LORA_S1_FREQ_MHZ,
    "lora_s2_freq":      config.LORA_S2_FREQ_MHZ,
    "vtx_s1_freq":       config.VTX_S1_FREQ_MHZ,
    "vtx_s2_freq":       config.VTX_S2_FREQ_MHZ,
    "vtx_s1_power":      0,   # SA index 0-3 (default 25mW)
    "vtx_s2_power":      0,   # SA index 0-3 (default 25mW)
    "last_session_name": "",
}


class SettingsManager:
    """
    Manages persistence of ground station session settings.

    Usage:
        sm = SettingsManager()
        d  = sm.load()          # Returns dict with all settings
        sm.save(updated_dict)   # Writes atomically
    """

    def __init__(self):
        self._data = dict(_DEFAULTS)

    # ------------------------------------------------------------------
    #  Public API
    # ------------------------------------------------------------------

    def load(self) -> dict:
        """
        Read settings.json.  Returns a dict with all known keys.
        Falls back to config.py defaults for any missing or corrupt key.
        """
        if not os.path.exists(_SETTINGS_FILE):
            self._data = dict(_DEFAULTS)
            return dict(self._data)

        try:
            with open(_SETTINGS_FILE, "r", encoding="utf-8") as f:
                raw = json.load(f)
        except (json.JSONDecodeError, OSError, ValueError):
            # Corrupt or unreadable — start from defaults
            self._data = dict(_DEFAULTS)
            return dict(self._data)

        # Merge: known keys from file, defaults for anything missing
        merged = dict(_DEFAULTS)
        for key in _DEFAULTS:
            if key in raw:
                merged[key] = raw[key]

        self._data = merged
        return dict(self._data)

    def save(self, settings_dict: dict) -> None:
        """
        Write settings_dict to settings.json atomically.
        Only keys listed in _DEFAULTS are written; unknown keys ignored.
        Uses write-to-temp-file + rename to prevent corruption on power loss.
        """
        to_write = {}
        for key in _DEFAULTS:
            to_write[key] = settings_dict.get(key, _DEFAULTS[key])

        # Atomic write: write to a temp file in the same directory,
        # then rename over the target.  Rename is atomic on POSIX.
        dir_path = os.path.dirname(_SETTINGS_FILE)
        try:
            fd, tmp_path = tempfile.mkstemp(
                dir=dir_path, prefix=".settings_", suffix=".tmp"
            )
            try:
                with os.fdopen(fd, "w", encoding="utf-8") as f:
                    json.dump(to_write, f, indent=2)
                    f.write("\n")
                os.replace(tmp_path, _SETTINGS_FILE)
            except Exception:
                try: os.unlink(tmp_path)
                except OSError: pass
                raise
        except OSError:
            # Non-fatal — settings simply won't be persisted this session
            pass

        self._data = dict(to_write)

    # ------------------------------------------------------------------
    #  Convenience properties (read from last load/save)
    # ------------------------------------------------------------------

    @property
    def lora_s1_freq(self) -> float:
        return float(self._data.get("lora_s1_freq", _DEFAULTS["lora_s1_freq"]))

    @property
    def lora_s2_freq(self) -> float:
        return float(self._data.get("lora_s2_freq", _DEFAULTS["lora_s2_freq"]))

    @property
    def vtx_s1_freq(self) -> int:
        return int(self._data.get("vtx_s1_freq", _DEFAULTS["vtx_s1_freq"]))

    @property
    def vtx_s2_freq(self) -> int:
        return int(self._data.get("vtx_s2_freq", _DEFAULTS["vtx_s2_freq"]))

    @property
    def vtx_s1_power(self) -> int:
        return int(self._data.get("vtx_s1_power", _DEFAULTS["vtx_s1_power"]))

    @property
    def vtx_s2_power(self) -> int:
        return int(self._data.get("vtx_s2_power", _DEFAULTS["vtx_s2_power"]))

    @property
    def last_session_name(self) -> str:
        return str(self._data.get("last_session_name", ""))

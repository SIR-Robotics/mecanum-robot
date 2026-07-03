Import("env")

from pathlib import Path

keys = {"DEVICE_DEVELOPER_ID", "DEVICE_ACCESS_TOKEN"}
env_file = Path(env["PROJECT_DIR"]) / ".env"

if env_file.exists():
    defines = []
    for line in env_file.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue

        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        if key in keys:
            defines.append((key, env.StringifyMacro(value)))

    env.Append(CPPDEFINES=defines)

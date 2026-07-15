Import("env")

from pathlib import Path

keys = {
    "DEVELOPER_ID": "DEVICE_DEVELOPER_ID",
    "ACCESS_TOKEN": "DEVICE_ACCESS_TOKEN",
    "DEVICE_DEVELOPER_ID": "DEVICE_DEVELOPER_ID",
    "DEVICE_ACCESS_TOKEN": "DEVICE_ACCESS_TOKEN",
    "DASHBOARD_BACKEND_HOST": "DASHBOARD_BACKEND_HOST",
}
numeric_keys = {"DASHBOARD_BACKEND_PORT"}
env_file = Path(env["PROJECT_DIR"]) / ".env"
found = set()

if env_file.exists():
    flags = []
    for line in env_file.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue

        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        if key in keys:
            define = keys[key]
            flags.append(f'-D{define}=\\"{value}\\"')
            found.add(define)
        elif key in numeric_keys:
            flags.append(f"-D{key}={value}")

    env.Append(BUILD_FLAGS=flags)

missing = {"DEVICE_DEVELOPER_ID", "DEVICE_ACCESS_TOKEN"} - found
if missing:
    raise RuntimeError(f"Missing {', '.join(sorted(missing))} in .env")

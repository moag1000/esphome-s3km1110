#!/usr/bin/env python3
"""Push a YAML config to the ESPHome Device Builder.

The dashboard's REST write endpoint is gone — POST /edit answers 405, as does
every other path that used to work. Writing goes over the websocket the web UI
itself uses: ws://<host>/ws, command "devices/update_config".

This matters because a local `esphome compile/upload` does not touch the
server's copy. Leave them apart and the next INSTALL pressed in the dashboard
rebuilds whatever the server still holds, quietly undoing the deployment.

Usage:
    python3 tools/push_config.py esp32-s3-mmwave.yaml [host:port]

Reads back what it wrote and fails loudly if it does not match.
"""
import asyncio
import hashlib
import sys

import websockets


async def push(path: str, host: str) -> int:
    content = open(path, encoding="utf-8").read()
    name = path.split("/")[-1]

    async with websockets.connect(f"ws://{host}/ws", max_size=None, open_timeout=15) as ws:
        await asyncio.wait_for(ws.recv(), timeout=5)  # server hello

        async def call(command, args, mid):
            import json
            await ws.send(json.dumps({"command": command, "args": args, "message_id": mid}))
            while True:
                r = json.loads(await asyncio.wait_for(ws.recv(), timeout=30))
                if str(r.get("message_id")) == str(mid):
                    return r

        before = (await call("devices/get_config", {"configuration": name}, 1)).get("result", "")
        if before == content:
            print(f"{name}: server already matches ({len(content)} bytes)")
            return 0

        print(f"{name}: server has {len(before)} bytes, pushing {len(content)}")
        await call("devices/update_config", {"configuration": name, "content": content}, 2)

        after = (await call("devices/get_config", {"configuration": name}, 3)).get("result", "")
        if after != content:
            print(f"{name}: MISMATCH after write — server holds {len(after)} bytes", file=sys.stderr)
            return 1
        print(f"{name}: pushed, sha {hashlib.sha256(after.encode()).hexdigest()[:12]}")
        return 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__, file=sys.stderr)
        sys.exit(2)
    host = sys.argv[2] if len(sys.argv) > 2 else "esphome.wildtierpark.local:6052"
    sys.exit(asyncio.run(push(sys.argv[1], host)))

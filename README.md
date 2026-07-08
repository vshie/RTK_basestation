# BlueOS RTK Base Station Extension

A BlueOS extension that turns a stationary [u-blox ZED-F9P](https://www.u-blox.com/en/product/zed-f9p-module)
into an **NTRIP server (source)**: it reads the RTCM3 corrections the F9P produces
in base-station mode over USB and pushes them to an NTRIP caster such as
[RTK2Go](http://rtk2go.com/), where rovers can consume them.

> This is the *opposite* direction to a rover NTRIP client. Here the device is the
> **source** of corrections, not the consumer. Nothing is sent to an autopilot.

## How it works

```
u-blox ZED-F9P (base mode, USB)  ──RTCM3──►  this extension  ──NTRIP v1 SOURCE──►  RTK2Go caster  ──►  rovers
        /dev/ttyACM0                          (parse/filter)      rtk2go.com:2101       mountpoint
```

1. On startup (if enabled) the extension opens the F9P serial port.
2. It parses the byte stream, keeps only valid RTCM3 frames (CRC-checked), and
   discards the NMEA the F9P also emits.
3. It connects to the caster, performs the NTRIP handshake, and streams the RTCM
   frames, reconnecting automatically with exponential backoff on any failure.

The F9P must already be configured as a base station (Survey-In or Fixed mode)
so that it outputs RTCM3 messages such as `1005/1006` (base position),
`1074/1084/1094/1124` (MSM4 observations) and `1230` (GLONASS biases).

## Features

- NTRIP **v1** (`SOURCE ... ICY 200 OK`, RTK2Go default) and **v2** (`HTTP POST`) push.
- RTCM3 framing with CRC-24Q validation; NMEA is filtered out automatically.
- Live status: caster response, bytes/messages pushed, per-type message counts.
- Decodes RTCM `1005/1006` to display the base station's surveyed position.
- Two built-in sanity checks in the UI:
  - **Test base station (serial)** — reads the F9P for a few seconds and reports the RTCM message types seen.
  - **Test caster connection** — performs the NTRIP handshake and reports whether the caster accepts the credentials.
- Persistent JSON configuration and auto-start on boot.
- u-blox serial device auto-detection via the stable `/dev/serial/by-id/` path.

## Configuration

Open the extension from the BlueOS sidebar and fill in:

| Field | Notes |
|-------|-------|
| Caster host / port | e.g. `rtk2go.com` / `2101` |
| Mountpoint | **Case sensitive**, must match your RTK2Go reservation |
| Mountpoint password | **Case sensitive** (RTK2Go "mount point password") |
| Username | Leave blank for NTRIP v1 (only used for v2 auth) |
| NTRIP version | `Rev1` for RTK2Go (default) |
| Serial device | Auto-detected u-blox by-id path if left at default |
| Serial baud | `115200` (ignored by USB CDC, kept for completeness) |
| Enable streaming | Starts immediately and on every boot |

All NTRIP fields (host, mountpoint, password) default to **blank** — nothing is
baked into the image. Whatever you enter in the UI is saved to
`config/rtk_config.json`, which is bind-mounted to
`/usr/blueos/extensions/rtk-basestation/config` on the host, so your settings
**persist across reboots and container restarts**. The only non-blank defaults
are the standard NTRIP port (`2101`) and the u-blox serial-device path.

### RTK2Go notes

- RTK2Go uses the **NTRIP Rev1** format (`SOURCE <password> /<mountpoint>`).
- Mountpoint and password are both case sensitive.
- A given mountpoint can only have one active server pushing to it at a time.

## Local development

```bash
pip install -r requirements.txt
cd app
python main.py            # serves http://localhost:8000
```

Options: `--host`, `--port`, `--config-file`, `--reload`.

## Building & publishing (GitHub Actions → Docker Hub)

This repo ships a workflow at `.github/workflows/deploy.yml` that uses the
official [BlueOS Deploy Extension action](https://github.com/BlueOS-community/Deploy-BlueOS-Extension)
to build multi-arch images and push them to Docker Hub on every push.

Configure the repository:

**Secrets** (`Settings → Secrets and variables → Actions → Secrets`):
- `DOCKER_USERNAME` — your Docker Hub username
- `DOCKER_PASSWORD` — a Docker Hub access token

**Variables** (`Settings → Secrets and variables → Actions → Variables`):
- `IMAGE_NAME` — Docker repo name, e.g. `blueos-rtk-basestation`
- `MY_NAME`, `MY_EMAIL` — author details
- `ORG_NAME`, `ORG_EMAIL` — maintainer details

The `Dockerfile` (`readme` / `links` labels) and `app/static/register_service`
point at `github.com/vshie/RTK_basestation`.

## Installing on BlueOS

Use the Extensions Manager → **Installed** → add a manual/development install:

- **Extension Identifier**: `vshie.rtk-basestation`
- **Extension Name**: `RTK Base Station`
- **Docker image**: `vshie/blueos-rtk-basestation`
- **Docker tag**: `main` (or a released version tag)
- **Custom settings**: paste the JSON below (this is the `Dockerfile`
  `permissions` label with the line-continuations removed).

```json
{
  "ExposedPorts": {
    "8000/tcp": {}
  },
  "HostConfig": {
    "Privileged": true,
    "Binds": [
      "/usr/blueos/extensions/rtk-basestation/config:/app/config",
      "/dev:/dev"
    ],
    "Dns": ["8.8.8.8", "1.1.1.1"],
    "ExtraHosts": ["host.docker.internal:host-gateway"],
    "PortBindings": {
      "8000/tcp": [
        {
          "HostPort": ""
        }
      ]
    }
  }
}
```

## Requirements

- BlueOS core >= 1.1
- A u-blox ZED-F9P (or compatible) configured as an RTK base station
- Internet access from the onboard computer to the caster

## License

GPLv3 — see [LICENSE](LICENSE).

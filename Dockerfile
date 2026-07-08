FROM python:3.11-slim

COPY app /app
RUN python -m pip install /app --extra-index-url https://www.piwheels.org/simple

EXPOSE 8000/tcp

LABEL version="1.0.0"

# Serial access requires privileged + /dev bind so the u-blox by-id symlink
# resolves inside the container. The config bind persists rtk_config.json.
LABEL permissions='\
{\
  "ExposedPorts": {\
    "8000/tcp": {}\
  },\
  "HostConfig": {\
    "Privileged": true,\
    "Binds":[\
      "/usr/blueos/extensions/rtk-basestation/config:/app/config",\
      "/dev:/dev"\
    ],\
    "Dns": ["8.8.8.8", "1.1.1.1"],\
    "ExtraHosts": ["host.docker.internal:host-gateway"],\
    "PortBindings": {\
      "8000/tcp": [\
        {\
          "HostPort": ""\
        }\
      ]\
    }\
  }\
}'

LABEL authors='[\
    {\
        "name": "Tony White",\
        "email": "you@example.com"\
    }\
]'

LABEL company='{\
        "about": "RTK base station corrections for BlueOS",\
        "name": "RTK Base Station",\
        "email": "you@example.com"\
    }'

LABEL type="device-integration"
LABEL tags='[\
    "positioning",\
    "navigation",\
    "data-collection"\
]'
LABEL readme='https://raw.githubusercontent.com/vshie/RTK_basestation/{tag}/README.md'
LABEL links='{\
        "source": "https://github.com/vshie/RTK_basestation"\
    }'
LABEL requirements="core >= 1.1"

WORKDIR /app
ENTRYPOINT ["python", "main.py", "--host", "0.0.0.0", "--config-file", "config/rtk_config.json"]

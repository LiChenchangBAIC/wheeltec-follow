"""Generate explicit unicast LAN settings for the navigation session."""
import ipaddress
import json
from pathlib import Path
import subprocess
import sys

peer = str(ipaddress.IPv4Address(sys.argv[1]))
routes = json.loads(subprocess.check_output(['ip', '-j', '-4', 'route', 'get', peer]))
local = str(ipaddress.IPv4Address(routes[0]['prefsrc']))
path = Path(sys.argv[2]) if len(sys.argv) > 2 else Path.home() / 'robot_ws/config/navigation-cyclone.xml'
path.parent.mkdir(parents=True, exist_ok=True)
path.write_text(f'''<?xml version="1.0"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces><NetworkInterface address="{local}" /></Interfaces>
      <AllowMulticast>false</AllowMulticast>
      <MaxMessageSize>1400B</MaxMessageSize>
      <FragmentSize>1024B</FragmentSize>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>64</MaxAutoParticipantIndex>
      <Peers><Peer Address="{local}"/><Peer Address="{peer}"/></Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
''')
print(path.as_uri())

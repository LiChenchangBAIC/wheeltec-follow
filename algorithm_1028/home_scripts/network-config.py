#!/usr/bin/env python3
"""Use the LAN route to the robot, excluding VPN and proxy interfaces."""
import ipaddress
import json
from pathlib import Path
import subprocess
import sys
import tempfile

robot = str(ipaddress.IPv4Address(sys.argv[1]))
routes = json.loads(subprocess.check_output(["ip", "-j", "-4", "route", "get", robot]))
local = str(ipaddress.IPv4Address(routes[0]["prefsrc"]))
config = Path(sys.argv[2]) if len(sys.argv) > 2 else Path.home() / "robot_ws" / "config" / "fastdds.xml"
xml = f'''<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>robot_lan_udp</transport_id>
      <type>UDPv4</type>
      <maxMessageSize>1400</maxMessageSize>
      <interfaceWhiteList>
        <address>127.0.0.1</address>
        <address>{local}</address>
      </interfaceWhiteList>
      <maxInitialPeersRange>64</maxInitialPeersRange>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="robot_lan" is_default_profile="true">
    <rtps>
      <builtin>
        <initialPeersList>
          <locator><udpv4><address>127.0.0.1</address></udpv4></locator>
          <locator><udpv4><address>{local}</address></udpv4></locator>
          <locator><udpv4><address>{robot}</address></udpv4></locator>
        </initialPeersList>
      </builtin>
      <useBuiltinTransports>false</useBuiltinTransports>
      <userTransports><transport_id>robot_lan_udp</transport_id></userTransports>
    </rtps>
  </participant>
</profiles>
'''
config.parent.mkdir(parents=True, exist_ok=True)
if not config.exists() or config.read_text() != xml:
    with tempfile.NamedTemporaryFile(mode="w", dir=config.parent, delete=False) as tmp:
        tmp.write(xml)
        temp_path = Path(tmp.name)
    temp_path.replace(config)
print(config)

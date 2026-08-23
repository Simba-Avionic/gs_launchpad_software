# Launchbox automatic startup

The two services replace manually running `run_nodes_as_screens.sh`. They run
the ROS 2 tanking and weight-module nodes directly, wait for their Raspberry Pi
UART devices and the launchbox Ethernet address, and restart a node if it exits
with an error. Waiting for Ethernet is important because ROS 2 DDS chooses its
network interfaces when each node starts.
The stop timeout prevents a stuck ROS serial-reader thread from blocking a
service restart indefinitely.

Install or update them on the launchbox:

```sh
sudo install -m 0644 systemd/gs_launchpad.service /etc/systemd/system/gs_launchpad.service
sudo install -m 0644 systemd/gs_weightmodule.service /etc/systemd/system/gs_weightmodule.service
sudo systemctl daemon-reload
sudo systemctl enable gs_launchpad.service gs_weightmodule.service
```

`enable` schedules the services for future boots but does not start them in the
current session. Start or restart them only when the connected tanking hardware
is in a known safe condition.

Inspect their state and logs with:

```sh
systemctl status gs_launchpad.service gs_weightmodule.service
journalctl -u gs_launchpad.service -u gs_weightmodule.service
```

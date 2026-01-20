#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gs_interfaces.msg import WirelessTelemetry, RadiolinkTelemetry

import librouteros
from typing import Dict, Any


class RadiolinkPublisher(Node):
    def __init__(self):
        super().__init__('radiolink_publisher')
        self.publisher_ = self.create_publisher(RadiolinkTelemetry, 'radiolink/telemetry', 10)
        self.timer = self.create_timer(1.0, self.publish_data)  # 1 Hz

        self.devices = {
            'launchpad_antenna': {'host': '192.168.10.142', 'user': 'admin', 'password': '', 'port': 8728},
            'mission_control_antenna': {'host': '192.168.10.141', 'user': 'admin', 'password': '', 'port': 8728}
        }

        self.connections: Dict[str, Any] = {}
        for name, cfg in self.devices.items():
            self.connections[name] = self.connect_to_device(**cfg)

    # ---------------- MikroTik API methods ----------------

    def connect_to_device(self, host, user, password, port):
        try:
            api = librouteros.connect(host=host, username=user, password=password, port=port, timeout=10)
            self.get_logger().info(f"Connected to {host}")
            return api
        except Exception as e:
            self.get_logger().error(f"Cannot connect to {host}: {e}")
            return None

    def ensure_connection(self, device_name: str):
        """Check if connection exists; reconnect if lost"""
        api = self.connections.get(device_name)
        try:
            # Test simple command
            if api:
                api(cmd="/system/identity/print", limit=1)
            else:
                raise Exception("No connection")
        except Exception:
            self.get_logger().warn(f"Reconnecting to {device_name}")
            cfg = self.devices[device_name]
            self.connections[device_name] = self.connect_to_device(**cfg)
        return self.connections[device_name]

    def parse_wireless_registration(self, registration: dict) -> dict:
        parsed = {}

        for k, v in registration.items():
            parsed[k.replace('-', '_')] = v

        sig = parsed.get('signal_strength', '')
        if '@' in sig:
            strength, _ = sig.split('@')
            try:
                parsed['signal_strength'] = int(strength)
            except:
                parsed['signal_strength'] = None
        else:
            try:
                parsed['signal_strength'] = int(sig)
            except:
                parsed['signal_strength'] = None

        for key in ['signal_to_noise', 'tx_ccq', 'rx_ccq']:
            try:
                parsed[key] = int(parsed.get(key))
            except:
                parsed[key] = 0

        for key in ['tx_rate', 'rx_rate']:
            val = parsed.get(key, '0')
            try:
                parsed[key] = float(val.replace('Mbps', '').strip())
            except:
                parsed[key] = 0.0

        return parsed


    def get_wireless_data(self, api) -> Dict[str, Any]:
        data: Dict[str, Any] = {}
        try:
            stats_detail = api(cmd="/interface/print", stats=True)
            wlan1_detail = next((i for i in stats_detail if i.get('name')=='wlan1'), None)
            if wlan1_detail:
                data.update({
                    'rx_bytes_mb': round(wlan1_detail.get('rx-byte',0)/(1024*1024),2),
                    'tx_bytes_mb': round(wlan1_detail.get('tx-byte',0)/(1024*1024),2),
                    'rx_bytes_kb': round(wlan1_detail.get('rx-byte',0)/(1024),2),
                    'tx_bytes_kb': round(wlan1_detail.get('tx-byte',0)/(1024),2),
                    'rx_packets': wlan1_detail.get('rx-packet',0),
                    'tx_packets': wlan1_detail.get('tx-packet',0),
                    'rx_drop': wlan1_detail.get('rx-drop',0),
                    'tx_drop': wlan1_detail.get('tx-drop',0),
                    'rx_error': wlan1_detail.get('rx-error',0),
                    'tx_error': wlan1_detail.get('tx-error',0),
                    'link_downs': wlan1_detail.get('link-downs',0)
                })
            traffic = api(cmd="/interface/monitor-traffic", **{"interface":"wlan1","once":True})
            traffic_detail = next((i for i in traffic if i.get('name')=='wlan1'), None)
            if traffic_detail:
                data.update({
                    'rx_mbps': round(traffic_detail.get('rx-bits-per-second',0)/1_000_000,2),
                    'tx_mbps': round(traffic_detail.get('tx-bits-per-second',0)/1_000_000,2),
                    'rx_kbps': round(traffic_detail.get('rx-bits-per-second',0)/1_000,2),
                    'tx_kbps': round(traffic_detail.get('tx-bits-per-second',0)/1_000,2),
                    'rx_packets_per_sec': float(traffic_detail.get('rx-packets-per-second',0)),
                    'tx_packets_per_sec': float(traffic_detail.get('tx-packets-per-second',0)),
                    'rx_drops_per_sec': float(traffic_detail.get('rx-drops-per-second',0)),
                    'tx_drops_per_sec': float(traffic_detail.get('tx-drops-per-second',0)),
                    'rx_errors_per_sec': float(traffic_detail.get('rx-errors-per-second',0)),
                    'tx_errors_per_sec': float(traffic_detail.get('tx-errors-per-second',0))
                })
            wireless_registration = api.path('interface','wireless','registration-table').select(
                'signal-strength','signal-to-noise','tx-rate','rx-rate','tx-ccq','rx-ccq'
            )
            client_data = {}
            for reg in wireless_registration:
                client_data = self.parse_wireless_registration(reg)
                break
            data.update(client_data)
        except Exception as e:
            data['error'] = str(e)
        
        # print(data)
        return data

    # ---------------- ROS2 publish method ----------------

    def publish_data(self):
        msg = RadiolinkTelemetry()
        msg.header.stamp = self.get_clock().now().to_msg()

        launchpad_api = self.ensure_connection('launchpad_antenna')
        mission_api = self.ensure_connection('mission_control_antenna')

        launchpad_data = self.get_wireless_data(launchpad_api) if launchpad_api else {}
        mission_data = self.get_wireless_data(mission_api) if mission_api else {}

        def dict_to_msg(data_dict):
            msg_w = WirelessTelemetry()
            for key, val in data_dict.items():
                if hasattr(msg_w, key):
                    setattr(msg_w, key, val)
            return msg_w

        msg.launchpad_antenna = dict_to_msg(launchpad_data)
        msg.mission_control_antenna = dict_to_msg(mission_data)

        self.publisher_.publish(msg)
        self.get_logger().info("Published RadiolinkTelemetry message")


def main(args=None):
    rclpy.init(args=args)
    node = RadiolinkPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

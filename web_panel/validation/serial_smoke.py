#!/usr/bin/env python3
"""Verify the actual ROS/pyserial bridge against an isolated PTY MCU peer."""
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

PANEL = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PANEL))
from server import stop_process
from serial_mcu import PtyMCU


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import JointState
    from nav_msgs.msg import Odometry

    mcu = PtyMCU(target_rad=.2)
    evidence = Path(os.environ.get('SCURM_VALIDATION_EVIDENCE', '/evidence/serial'))
    evidence.mkdir(parents=True, exist_ok=True)
    configuration = evidence/'virtual_serial_config.json'
    configuration.write_text(json.dumps({'version':1, 'yaw':{'target_rad':.2}, 'hardware':{
        'transport':'serial_jsonl_v1', 'serial_port':mcu.port, 'baud':460800,
        'yaw_encoder_sign':mcu.sign, 'yaw_encoder_zero_rad':mcu.zero, 'command_timeout_s':.3}}))
    rclpy.init()
    node = Node('serial_protocol_validation_observer')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    latest = {}
    def received(key, msg):
        latest[key] = (time.monotonic(), msg)
    node.create_subscription(JointState, '/yaw/joint_states', lambda msg: received('yaw', msg), 10)
    node.create_subscription(Odometry, '/wheel/odom', lambda msg: received('odom', msg), 10)
    publisher = node.create_publisher(Twist, '/cmd_vel', 1)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    log = (evidence/'serial-bridge.log').open('wb')
    bridge = subprocess.Popen([sys.executable, '-u', str(PANEL/'chassis_bridge.py'),
        '--config', str(configuration)], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    report = {'simulation_only':True, 'transport':'real pyserial over POSIX PTY', 'steps':[]}

    def wait_for(check, description, timeout=5):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            if mcu.error:
                raise RuntimeError(mcu.error)
            if bridge.poll() is not None:
                raise RuntimeError('serial bridge exited: see serial-bridge.log')
            if check():
                print('PASS: '+description, flush=True)
                return
            time.sleep(.02)
        raise TimeoutError(description)

    def command(values, duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = values
            publisher.publish(msg)
            time.sleep(.04)

    def zero(packet):
        return all(packet[k] == 0. for k in ('vx','vy','wz'))

    def blocked_since(started):
        packets = [p for stamp, p in mcu.commands if stamp > started+.5]
        assert packets and all(zero(p) for p in packets), packets

    try:
        wait_for(lambda: 'yaw' in latest and 'odom' in latest, 'serial feedback becomes ROS yaw and odometry')
        assert abs(latest['yaw'][1].position[0]-.2) < 1e-9
        wait_for(lambda: mcu.commands and abs(mcu.commands[-1][1]['yaw_target_encoder_rad']-.5) < 1e-9,
                 'yaw sign/zero conversion reaches UART bytes')
        report['steps'].append('serial_to_ros_feedback_and_encoder_calibration')
        began = time.monotonic()
        command((.12, -.08, .2), .7)
        packets = [p for stamp, p in mcu.commands if stamp > began]
        assert any((p['vx'],p['vy'],p['wz']) == (.12,-.08,.2) for p in packets)
        assert latest['odom'][1].twist.twist.linear.y < -.01
        report['steps'].append('vx_vy_wz_serial_round_trip')
        ended = time.monotonic()
        wait_for(lambda: time.monotonic()-ended > .4 and zero(mcu.commands[-1][1]),
                 'ROS command timeout sends serial zero')
        report['steps'].append('command_timeout_serial_zero')
        mcu.repeat_sequence = True
        began = time.monotonic()
        command((.1, 0., 0.), 1.)
        blocked_since(began)
        report['steps'].append('duplicate_feedback_cannot_refresh_watchdog')
        mcu.repeat_sequence = False
        command((.1, 0., 0.), .5)
        wait_for(lambda: mcu.commands[-1][1]['vx'] == .1, 'new feedback restores serial forwarding')
        mcu.feedback_enabled = False
        began = time.monotonic()
        command((.1, 0., 0.), 1.)
        blocked_since(began)
        report['steps'].append('feedback_loss_serial_zero')
        mcu.feedback_enabled = True
        mcu.inject_bad_frame = True
        command((.1, 0., 0.), .6)
        wait_for(lambda: mcu.commands[-1][1]['vx'] == .1, 'malformed and split frames recover')
        report['steps'].append('split_and_malformed_frame_recovery')
        stopped_at = time.monotonic()
        stop_process(bridge)
        end = time.monotonic()+2
        while not (mcu.commands[-1][0] > stopped_at and zero(mcu.commands[-1][1])) and time.monotonic() < end:
            time.sleep(.01)
        assert mcu.commands[-1][0] > stopped_at and zero(mcu.commands[-1][1])
        assert bridge.returncode == 0, bridge.returncode
        report['steps'].append('bridge_exit_sends_final_serial_zero')
        report['passed'] = True
    except Exception as exc:
        report.update(passed=False, error=repr(exc))
        raise
    finally:
        stop_process(bridge)
        executor.shutdown()
        thread.join(timeout=3)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        report['commands_received'] = len(mcu.commands)
        (evidence/'serial-integration.json').write_text(json.dumps(report, indent=2)+'\n')
        mcu.close()
        log.close()
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()

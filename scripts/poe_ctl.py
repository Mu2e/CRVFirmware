#!/usr/bin/env python3
"""CLI tool for Microsemi PD-9524GC-10G PoE injector monitoring and control."""

import argparse
import json
import os
import re
import sys
import time

import requests
import yaml

NUM_PORTS = 24
CONFIG_DEFAULT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "poe_config.yaml")


def load_config(path=None):
    path = path or CONFIG_DEFAULT
    with open(path) as f:
        return yaml.safe_load(f)


class PoeInjector:
    def __init__(self, host, username="admin", password="password", injector_id=None):
        self.host = host
        self.auth = (username, password)
        self.injector_id = injector_id
        self.base = f"http://{host}"
        self.timeout = 10

    def _get(self, path, auth=False):
        url = f"{self.base}{path}"
        kwargs = {"timeout": self.timeout}
        if auth:
            kwargs["auth"] = self.auth
        resp = requests.get(url, **kwargs)
        resp.raise_for_status()
        return resp.text

    def get_status(self):
        html = self._get("/web/P24/view/view_system_status.htm")

        power_row_match = re.search(
            r'<tr[^>]*>\s*((?:<td[^>]*>[^<]*</td>\s*)+)<td[^>]*>Power \(W\)</td>',
            html,
        )
        port_power_values = []
        if power_row_match:
            cells = re.findall(r'<td[^>]*>\s*([^<]*?)\s*</td>', power_row_match.group(1))
            for cell in cells:
                cell = cell.strip()
                if cell and cell != "&nbsp;":
                    try:
                        port_power_values.append(float(cell))
                    except ValueError:
                        pass

        def extract_system_value(label, text):
            m = re.search(label + r'</span></td>\s*<td[^>]*>\s*<span[^>]*>([^<]+)', text)
            return m.group(1).strip() if m else None

        def extract_float(label, text):
            val = extract_system_value(label, text)
            if val:
                try:
                    return float(val)
                except ValueError:
                    return None
            return None

        visible_block = re.search(
            r'<div style="visibility:visible;">(.*?)</div>\s*\n\s*</div>',
            html, re.DOTALL,
        )
        status_html = visible_block.group(1) if visible_block else html

        total_power = extract_float(r'Total Power Consumption \(Watt\)', status_html)
        max_power = extract_float(r'Maximum available Power \(Watt\)', status_html)
        voltage = extract_float(r'System Voltage \(Volt\)', status_html)
        temperature = extract_float(r'Temperature \(F\)', status_html)

        status_text = None
        m = re.search(r'Midspan Status</span></td>\s*<td[^>]*>.*?<span[^>]*>(\w+)</span>', status_html)
        if m:
            status_text = m.group(1)

        port_led_images = re.findall(r'<img src="/web/image/(led_\w+\.(?:jpg|gif))"', html)
        port_images = re.findall(r'<img src="/web/image/(port_\w+\.jpg)"', html)

        ports = {}
        for i in range(NUM_PORTS):
            port_num = i + 1
            port = {"port": port_num}
            if i < len(port_power_values):
                port["power_w"] = port_power_values[i]
            if i < len(port_images):
                img = port_images[i]
                if "enable" in img:
                    port["enabled"] = True
                elif "disable" in img:
                    port["enabled"] = False
            if i < len(port_led_images):
                led = port_led_images[i]
                if "green" in led:
                    port["status"] = "on"
                elif "red" in led:
                    port["status"] = "fault"
                elif "orange" in led or "amber" in led:
                    port["status"] = "warning"
                else:
                    port["status"] = "off"
            ports[port_num] = port

        return {
            "host": self.host,
            "injector_id": self.injector_id,
            "total_power_w": total_power,
            "max_power_w": max_power,
            "voltage_v": voltage,
            "temperature_f": temperature,
            "status": status_text,
            "ports": ports,
        }

    def get_port_detail(self, port_num):
        port_index = port_num - 1
        html = self._get(f"/web/view/port_info.htm?PORT={port_index}")

        fields = {}
        for label, key in [
            ("Status", "status"),
            ("Operation mode", "operation_mode"),
            ("Power \\(W\\)", "power"),
            ("Max power \\(W\\)", "max_power_w"),
            ("Priority", "priority"),
            ("Terminal type / Description", "description"),
            ("Class", "class"),
        ]:
            m = re.search(
                rf'{label}</span></td>\s*<td><span[^>]*>(.*?)</span>',
                html,
            )
            if m:
                val = m.group(1).strip()
                fields[key] = val

        if "power" in fields:
            m = re.match(r'([0-9.]+)', fields["power"])
            if m:
                fields["power_w"] = float(m.group(1))
            fields["power_detail"] = fields.pop("power")

        if "max_power_w" in fields:
            try:
                fields["max_power_w"] = float(fields["max_power_w"])
            except ValueError:
                pass

        fields["port"] = port_num
        fields["host"] = self.host
        fields["injector_id"] = self.injector_id
        return fields

    def get_port_states(self):
        html = self._get("/web/P24/config/cfg_ports_en_dis.htm", auth=True)
        states = []
        for i in range(NUM_PORTS):
            name = f"P{i:02d}_EN_DIS"
            pattern = rf'name="{name}"[^>]*checked'
            states.append(bool(re.search(pattern, html)))
        return states

    def set_port_states(self, states):
        data = {}
        for i in range(NUM_PORTS):
            if states[i]:
                data[f"P{i:02d}_EN_DIS"] = "1"
        data["SAVE"] = "1"
        url = f"{self.base}/web/P24/config/cfg_ports_en_dis_form"
        resp = requests.post(url, data=data, auth=self.auth, timeout=self.timeout)
        resp.raise_for_status()

    def enable_port(self, port_num):
        states = self.get_port_states()
        if port_num == "all":
            states = [True] * NUM_PORTS
        else:
            states[port_num - 1] = True
        self.set_port_states(states)

    def disable_port(self, port_num):
        states = self.get_port_states()
        if port_num == "all":
            states = [False] * NUM_PORTS
        else:
            states[port_num - 1] = False
        self.set_port_states(states)

    def cycle_port(self, port_num, delay=5):
        self.disable_port(port_num)
        time.sleep(delay)
        self.enable_port(port_num)


def get_injectors(config, injector_id):
    injectors = []
    for id_str, inj_cfg in config["injectors"].items():
        id_val = str(id_str)
        if injector_id == "all" or id_val == str(injector_id):
            injectors.append(
                PoeInjector(
                    host=inj_cfg["host"],
                    username=inj_cfg.get("username", "admin"),
                    password=inj_cfg.get("password", "password"),
                    injector_id=id_val,
                )
            )
    if not injectors:
        print(f"Error: no injector found with id '{injector_id}'", file=sys.stderr)
        sys.exit(1)
    return injectors


def parse_port_arg(port_str):
    if port_str.lower() == "all":
        return "all"
    try:
        p = int(port_str)
    except ValueError:
        print(f"Error: invalid port '{port_str}'. Use 1-{NUM_PORTS} or 'all'.", file=sys.stderr)
        sys.exit(1)
    if p < 1 or p > NUM_PORTS:
        print(f"Error: port must be 1-{NUM_PORTS}, got {p}.", file=sys.stderr)
        sys.exit(1)
    return p


def format_status_column(status):
    lines = []
    inj_id = status["injector_id"] or status["host"]
    lines.append(f"Injector {inj_id} ({status['host']})")
    lines.append(f"  {status['status']}  {status['total_power_w']}W/{status['max_power_w']}W"
                 f"  {status['voltage_v']}V  {status['temperature_f']}F")
    lines.append("")
    lines.append("  Port | Power(W) | Ena | Status")
    lines.append("  " + "-" * 30)
    for port_num in sorted(status["ports"]):
        p = status["ports"][port_num]
        power = f"{p.get('power_w', '?'):>8}" if isinstance(p.get("power_w"), float) else f"{'?':>8}"
        enabled = "yes" if p.get("enabled") else "no "
        port_status = p.get("status", "?")
        lines.append(f"  {port_num:>4} | {power} | {enabled} | {port_status}")
    return lines


def format_side_by_side(results, gap=4):
    if not results:
        return []
    columns = [format_status_column(r) for r in results]
    max_lines = max(len(c) for c in columns)
    for c in columns:
        c.extend([""] * (max_lines - len(c)))
    col_widths = [max(len(line) for line in c) for c in columns]
    separator = " " * gap
    merged = []
    for row_idx in range(max_lines):
        parts = [columns[ci][row_idx].ljust(col_widths[ci]) for ci in range(len(columns))]
        merged.append(separator.join(parts))
    return merged


def print_status(results):
    if len(results) > 1:
        print()
        for line in format_side_by_side(results):
            print(line)
    else:
        for r in results:
            print()
            for line in format_status_column(r):
                print(line)


def print_detail(detail):
    inj_id = detail.get("injector_id") or detail["host"]
    print(f"\n=== Injector {inj_id} - Port {detail['port']} ===")
    for key in ["status", "operation_mode", "power_detail", "power_w",
                 "max_power_w", "priority", "class", "description"]:
        if key in detail:
            label = key.replace("_", " ").title()
            print(f"  {label}: {detail[key]}")


def cmd_status(args):
    config = load_config(args.config)
    injectors = get_injectors(config, args.id)
    results = []
    for inj in injectors:
        try:
            status = inj.get_status()
            results.append(status)
        except requests.RequestException as e:
            print(f"Error contacting injector {inj.injector_id} ({inj.host}): {e}", file=sys.stderr)
    if args.json:
        print(json.dumps(results, indent=2))
    else:
        print_status(results)


def cmd_detail(args):
    config = load_config(args.config)
    injectors = get_injectors(config, args.id)
    port = parse_port_arg(args.port)
    results = []
    for inj in injectors:
        try:
            if port == "all":
                for p in range(1, NUM_PORTS + 1):
                    results.append(inj.get_port_detail(p))
            else:
                results.append(inj.get_port_detail(port))
        except requests.RequestException as e:
            print(f"Error contacting injector {inj.injector_id} ({inj.host}): {e}", file=sys.stderr)
    if args.json:
        print(json.dumps(results, indent=2))
    else:
        for detail in results:
            print_detail(detail)


def cmd_on(args):
    config = load_config(args.config)
    injectors = get_injectors(config, args.id)
    port = parse_port_arg(args.port)
    for inj in injectors:
        try:
            inj.enable_port(port)
            label = f"all ports" if port == "all" else f"port {port}"
            print(f"Injector {inj.injector_id}: enabled {label}")
        except requests.RequestException as e:
            print(f"Error contacting injector {inj.injector_id} ({inj.host}): {e}", file=sys.stderr)


def cmd_off(args):
    config = load_config(args.config)
    injectors = get_injectors(config, args.id)
    port = parse_port_arg(args.port)
    for inj in injectors:
        try:
            inj.disable_port(port)
            label = f"all ports" if port == "all" else f"port {port}"
            print(f"Injector {inj.injector_id}: disabled {label}")
        except requests.RequestException as e:
            print(f"Error contacting injector {inj.injector_id} ({inj.host}): {e}", file=sys.stderr)


def cmd_cycle(args):
    config = load_config(args.config)
    injectors = get_injectors(config, args.id)
    port = parse_port_arg(args.port)
    delay = args.delay if args.delay is not None else config.get("defaults", {}).get("cycle_delay", 5)
    for inj in injectors:
        try:
            label = f"all ports" if port == "all" else f"port {port}"
            print(f"Injector {inj.injector_id}: cycling {label} (delay={delay}s)...")
            inj.cycle_port(port, delay=delay)
            print(f"Injector {inj.injector_id}: {label} back on")
        except requests.RequestException as e:
            print(f"Error contacting injector {inj.injector_id} ({inj.host}): {e}", file=sys.stderr)


def cmd_monitor(args):
    config = load_config(args.config)
    injectors = get_injectors(config, args.id)
    interval = args.interval
    try:
        while True:
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            results = []
            errors = []
            for inj in injectors:
                try:
                    status = inj.get_status()
                    results.append(status)
                except requests.RequestException as e:
                    errors.append(f"Injector {inj.injector_id}: {e}")
            if args.json:
                record = {"timestamp": timestamp, "injectors": results}
                print(json.dumps(record))
                sys.stdout.flush()
            else:
                lines = []
                lines.append(f"PoE Monitor  |  Last update: {timestamp}  |  Refresh: {interval}s  |  Ctrl-C to stop")
                lines.append("")
                lines.extend(format_side_by_side(results) if len(results) > 1 else
                             format_status_column(results[0]) if results else [])
                for err in errors:
                    lines.append(f"  ERROR: {err}")
                output = "\n".join(lines)
                print(f"\033[2J\033[H{output}", flush=True)
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\033[2J\033[H", end="", flush=True)
        print("Monitor stopped.")


def add_common_args(parser):
    parser.add_argument(
        "--config", default=CONFIG_DEFAULT,
        help=f"path to config YAML (default: {CONFIG_DEFAULT})"
    )
    parser.add_argument(
        "--id", default="all",
        help="injector ID from config, or 'all' (default: all)"
    )
    parser.add_argument("--json", action="store_true", help="output as JSON")


def create_parser():
    parser = argparse.ArgumentParser(
        description="Monitor and control PD-9524GC-10G PoE injectors"
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p_status = sub.add_parser("status", help="show per-port power and system status")
    add_common_args(p_status)

    p_detail = sub.add_parser("detail", help="show detailed info for a port")
    p_detail.add_argument("port", help="port number (1-24) or 'all'")
    add_common_args(p_detail)

    p_on = sub.add_parser("on", help="enable a port")
    p_on.add_argument("port", nargs="?", default="all", help="port number (1-24) or 'all' (default: all)")
    add_common_args(p_on)

    p_off = sub.add_parser("off", help="disable a port")
    p_off.add_argument("port", nargs="?", default="all", help="port number (1-24) or 'all' (default: all)")
    add_common_args(p_off)

    p_cycle = sub.add_parser("cycle", help="power-cycle a port (off, wait, on)")
    p_cycle.add_argument("port", nargs="?", default="all", help="port number (1-24) or 'all' (default: all)")
    p_cycle.add_argument("--delay", type=float, default=None,
                         help="seconds to wait between off and on (default: from config)")
    add_common_args(p_cycle)

    p_mon = sub.add_parser("monitor", help="poll status periodically")
    p_mon.add_argument("--interval", type=float, default=60,
                        help="polling interval in seconds (default: 60)")
    add_common_args(p_mon)

    return parser


def main():
    parser = create_parser()
    args = parser.parse_args()
    commands = {
        "status": cmd_status,
        "detail": cmd_detail,
        "on": cmd_on,
        "off": cmd_off,
        "cycle": cmd_cycle,
        "monitor": cmd_monitor,
    }
    commands[args.command](args)


if __name__ == "__main__":
    main()

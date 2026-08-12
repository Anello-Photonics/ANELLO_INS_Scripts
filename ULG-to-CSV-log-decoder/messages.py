#!/usr/bin/env python3
"""
PX4 ULog Watchdog and CPU Analyzer

Features:
    - Prompts for a PX4 .ulg file.
    - Runs the standard verbose ulog_info output.
    - Extracts watchdog top and performance-counter snapshots.
    - Plots CPU usage and idle percentage across the entire log.
    - Highlights periods where CPU idle falls below a threshold.
    - Plots RAM usage when available.
    - Exports cpuload data to CSV.
    - Writes a watchdog summary text file.
    - Attempts to mark ULog dropouts when timestamp information is available.

Dependencies:
    python -m pip install pyulog matplotlib

Usage:
    python ulog_watchdog_info.py
"""

from __future__ import annotations

import csv
import os
import re
import sys
import traceback
from pathlib import Path
from typing import Any, Iterable

try:
    import matplotlib.pyplot as plt
except ImportError as exc:
    raise SystemExit(
        "matplotlib is not installed.\n\n"
        "Install dependencies using:\n"
        "    python -m pip install pyulog matplotlib"
    ) from exc


# Highlight CPU idle below this percentage.
LOW_IDLE_THRESHOLD_PERCENT = 5.0


def pause_before_exit() -> None:
    """Keep the console open when launched by double-clicking."""
    try:
        input("\nPress Enter to close...")
    except EOFError:
        pass


def select_ulg_file() -> str:
    """Open a file picker, with a console path fallback."""
    try:
        import tkinter as tk
        from tkinter import filedialog

        root = tk.Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        root.update()

        filename = filedialog.askopenfilename(
            parent=root,
            title="Select PX4 ULog File",
            filetypes=[
                ("PX4 ULog files", "*.ulg"),
                ("All files", "*.*"),
            ],
        )

        root.destroy()

        if filename:
            return filename

        print("No file was selected.")

    except Exception as exc:
        print(f"File picker could not be opened: {exc}")

    print("\nPaste the full path to the .ulg file.")
    return input("ULog path: ").strip().strip('"')


def print_heading(text: str, character: str = "=") -> None:
    print()
    print(text)
    print(character * len(text))


def run_ulog_info(ulg_path: str) -> None:
    """Run the standard pyulog ulog_info command."""
    try:
        from pyulog import info
    except ImportError as exc:
        raise RuntimeError(
            "pyulog is not installed.\n\n"
            "Install it using:\n"
            "    python -m pip install pyulog matplotlib"
        ) from exc

    original_argv = sys.argv.copy()

    try:
        sys.argv = ["ulog_info", ulg_path, "-v"]
        info.main()
    finally:
        sys.argv = original_argv


def flatten_nested_value(value: Any) -> list[str]:
    """
    Convert nested pyulog information-message values into strings.

    Values can contain strings, bytes, lists, tuples, or nested combinations.
    """
    output: list[str] = []

    def walk(item: Any) -> None:
        if item is None:
            return

        if isinstance(item, bytes):
            output.append(item.decode("utf-8", errors="replace"))
            return

        if isinstance(item, str):
            output.append(item)
            return

        if isinstance(item, (list, tuple)):
            for child in item:
                walk(child)
            return

        output.append(str(item))

    walk(value)
    return output


def get_multiple_info_lines(ulog: Any, key: str) -> list[str]:
    """Retrieve normalized lines from msg_info_multiple_dict."""
    value = ulog.msg_info_multiple_dict.get(key)

    if value is None:
        return []

    chunks = flatten_nested_value(value)
    lines: list[str] = []

    for chunk in chunks:
        lines.extend(chunk.splitlines())

    return [line.rstrip() for line in lines if line.strip()]


def parse_percentage(line: str, label: str) -> float | None:
    """
    Parse a percentage from a line such as:

        CPU usage: 98.64% tasks, 1.36% sched, 0.00% idle
    """
    match = re.search(
        rf"([-+]?\d+(?:\.\d+)?)%\s*{re.escape(label)}",
        line,
        flags=re.IGNORECASE,
    )

    if not match:
        return None

    return float(match.group(1))


def parse_top_task_line(line: str) -> dict[str, Any] | None:
    """
    Parse one task row from PX4 top output.

    Example:
        1525 logger  8  2.598  3948/4920 234 (234) RUN 4
    """
    pattern = re.compile(
        r"^\s*"
        r"(?P<pid>\d+)\s+"
        r"(?P<command>.+?)\s+"
        r"(?P<cpu_ms>\d+)\s+"
        r"(?P<cpu_percent>\d+(?:\.\d+)?)\s+"
        r"(?P<used>\d+)\s*/\s*(?P<stack>\d+)\s+"
        r"(?P<priority>\d+)"
    )

    match = pattern.match(line)

    if not match:
        return None

    used = int(match.group("used"))
    stack = int(match.group("stack"))

    return {
        "pid": int(match.group("pid")),
        "command": match.group("command").strip(),
        "cpu_ms": int(match.group("cpu_ms")),
        "cpu_percent": float(match.group("cpu_percent")),
        "stack_used": used,
        "stack_total": stack,
        "stack_percent": (used / stack * 100.0) if stack else 0.0,
    }


def extract_top_summary(lines: list[str]) -> dict[str, Any]:
    """Extract CPU, task, process, DMA, and uptime information."""
    result: dict[str, Any] = {
        "tasks": [],
        "task_cpu": None,
        "scheduler_cpu": None,
        "idle_cpu": None,
        "processes": None,
        "dma": None,
        "uptime": None,
    }

    for line in lines:
        stripped = line.strip()

        if stripped.startswith("PID COMMAND"):
            continue

        task = parse_top_task_line(line)

        if task:
            result["tasks"].append(task)
            continue

        if stripped.startswith("CPU usage:"):
            result["task_cpu"] = parse_percentage(stripped, "tasks")
            result["scheduler_cpu"] = parse_percentage(stripped, "sched")
            result["idle_cpu"] = parse_percentage(stripped, "idle")

        elif stripped.startswith("Processes:"):
            result["processes"] = stripped

        elif stripped.startswith("DMA Memory:"):
            result["dma"] = stripped

        elif stripped.startswith("Uptime:"):
            result["uptime"] = stripped

    return result


def parse_counter_line(line: str) -> dict[str, Any]:
    """Parse useful fields from a PX4 performance-counter line."""
    name, separator, details = line.partition(":")

    result: dict[str, Any] = {
        "name": name.strip(),
        "details": details.strip() if separator else "",
        "events": None,
        "max_us": None,
        "avg_us": None,
    }

    event_match = re.search(r"(\d+)\s+events?", details)

    if event_match:
        result["events"] = int(event_match.group(1))

    max_match = re.search(r"\bmax\s+(\d+)us", details)

    if max_match:
        result["max_us"] = int(max_match.group(1))

    avg_match = re.search(r"(\d+(?:\.\d+)?)us\s+avg", details)

    if avg_match:
        result["avg_us"] = float(avg_match.group(1))

    return result


def is_interesting_counter(counter: dict[str, Any]) -> bool:
    """Select counters useful during watchdog diagnosis."""
    name = counter["name"].lower()
    events = counter["events"]
    max_us = counter["max_us"]

    important_terms = (
        "error",
        "overflow",
        "missed",
        "gap",
        "reset",
        "bad transfer",
        "bad register",
        "checksum bad",
        "dropout",
        "logger_sd_write",
        "logger_sd_fsync",
        "ekf update",
        "loop duration",
        "poll error",
        "read error",
        "write error",
        "com_err",
        "fifo empty",
        "send_bytes",
        "forwarding",
    )

    if any(term in name for term in important_terms):
        if events is None:
            return True

        if events > 0:
            return True

        if name.startswith("logger_sd_"):
            return True

    # Highlight operations that took 50 ms or longer.
    if max_us is not None and max_us >= 50_000:
        return True

    return False


def get_dataset_or_none(
    ulog: Any,
    dataset_name: str,
    multi_instance: int = 0,
) -> Any | None:
    """Return a ULog dataset without raising if it does not exist."""
    try:
        return ulog.get_dataset(dataset_name, multi_instance)
    except (KeyError, IndexError, ValueError):
        return None


def normalize_fraction_or_percent(values: Iterable[Any]) -> list[float]:
    """
    Normalize a PX4 field that may use fractions or percentages.

    Examples:
        0.75 becomes 75.0
        75.0 remains 75.0
    """
    output = [float(value) for value in values]

    valid_values = [
        value
        for value in output
        if value == value  # NaN-safe check
    ]

    if not valid_values:
        return output

    maximum = max(valid_values)

    # PX4 cpuload.load and ram_usage are normally fractions from 0 to 1.
    if maximum <= 1.5:
        return [value * 100.0 for value in output]

    return output


def moving_average(values: list[float], window: int) -> list[float]:
    """Calculate a simple moving average without requiring NumPy."""
    if window <= 1 or len(values) < window:
        return values.copy()

    result: list[float] = []
    running_sum = 0.0

    for index, value in enumerate(values):
        running_sum += value

        if index >= window:
            running_sum -= values[index - window]

        divisor = min(index + 1, window)
        result.append(running_sum / divisor)

    return result


def find_low_idle_ranges(
    times_hours: list[float],
    idle_percent: list[float],
    threshold: float,
) -> list[tuple[float, float]]:
    """
    Find continuous ranges where idle CPU is below the configured threshold.
    """
    ranges: list[tuple[float, float]] = []
    range_start: float | None = None

    for time_value, idle_value in zip(times_hours, idle_percent):
        is_low = idle_value < threshold

        if is_low and range_start is None:
            range_start = time_value

        elif not is_low and range_start is not None:
            ranges.append((range_start, time_value))
            range_start = None

    if range_start is not None and times_hours:
        ranges.append((range_start, times_hours[-1]))

    return ranges


def extract_dropout_markers(
    ulog: Any,
    log_start_timestamp: int,
) -> list[tuple[float, float]]:
    """
    Attempt to extract dropout timestamps and durations.

    pyulog versions differ in how dropouts are represented. Some versions
    retain only durations, which cannot be positioned on a time plot.

    Returns:
        List of tuples:
            (elapsed_hours, duration_ms)
    """
    markers: list[tuple[float, float]] = []

    for dropout in getattr(ulog, "dropouts", []):
        timestamp = None
        duration = None

        if isinstance(dropout, dict):
            timestamp = dropout.get("timestamp")
            duration = dropout.get("duration")

        elif isinstance(dropout, (list, tuple)):
            if len(dropout) >= 2:
                timestamp = dropout[0]
                duration = dropout[1]

        else:
            timestamp = getattr(dropout, "timestamp", None)
            duration = getattr(dropout, "duration", None)

        if timestamp is None or duration is None:
            continue

        try:
            elapsed_hours = (
                float(timestamp) - float(log_start_timestamp)
            ) / 3_600_000_000.0

            markers.append((elapsed_hours, float(duration)))

        except (TypeError, ValueError):
            continue

    return markers


def export_cpuload_csv(
    output_path: Path,
    timestamps_us: list[int],
    elapsed_seconds: list[float],
    cpu_percent: list[float],
    idle_percent: list[float],
    ram_percent: list[float] | None,
) -> None:
    """Export cpuload data to CSV."""
    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)

        header = [
            "timestamp_us",
            "elapsed_seconds",
            "elapsed_hours",
            "cpu_usage_percent",
            "cpu_idle_percent",
        ]

        if ram_percent is not None:
            header.append("ram_usage_percent")

        writer.writerow(header)

        for index in range(len(timestamps_us)):
            row = [
                timestamps_us[index],
                elapsed_seconds[index],
                elapsed_seconds[index] / 3600.0,
                cpu_percent[index],
                idle_percent[index],
            ]

            if ram_percent is not None:
                row.append(ram_percent[index])

            writer.writerow(row)


def plot_cpu_usage(
    output_path: Path,
    elapsed_hours: list[float],
    cpu_percent: list[float],
    idle_percent: list[float],
    dropout_markers: list[tuple[float, float]],
) -> dict[str, Any]:
    """Plot CPU usage and CPU idle percentage for the entire log."""
    smoothed_cpu = moving_average(cpu_percent, window=5)
    smoothed_idle = moving_average(idle_percent, window=5)

    low_idle_ranges = find_low_idle_ranges(
        elapsed_hours,
        idle_percent,
        LOW_IDLE_THRESHOLD_PERCENT,
    )

    figure, axis = plt.subplots(figsize=(14, 7))

    axis.plot(
        elapsed_hours,
        smoothed_cpu,
        label="CPU usage",
        linewidth=1.0,
    )

    axis.plot(
        elapsed_hours,
        smoothed_idle,
        label="CPU idle",
        linewidth=1.0,
    )

    axis.axhline(
        LOW_IDLE_THRESHOLD_PERCENT,
        linestyle="--",
        linewidth=1.0,
        label=f"{LOW_IDLE_THRESHOLD_PERCENT:.1f}% idle threshold",
    )

    first_low_range = True

    for start_hour, end_hour in low_idle_ranges:
        axis.axvspan(
            start_hour,
            end_hour,
            alpha=0.18,
            label="Low CPU idle" if first_low_range else None,
        )
        first_low_range = False

    first_dropout = True

    for elapsed_hour, duration_ms in dropout_markers:
        axis.axvline(
            elapsed_hour,
            linestyle=":",
            linewidth=1.0,
            label="ULog dropout" if first_dropout else None,
        )

        first_dropout = False

        axis.annotate(
            f"{duration_ms:.0f} ms",
            xy=(elapsed_hour, 95.0),
            xytext=(4, 0),
            textcoords="offset points",
            rotation=90,
            va="top",
            fontsize=8,
        )

    axis.set_title("PX4 CPU usage over the entire log")
    axis.set_xlabel("Elapsed log time (hours)")
    axis.set_ylabel("CPU percentage")
    axis.set_ylim(0.0, 100.0)
    axis.grid(True, alpha=0.3)
    axis.legend(loc="best")

    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    plt.close(figure)

    low_idle_sample_count = sum(
        idle < LOW_IDLE_THRESHOLD_PERCENT
        for idle in idle_percent
    )

    return {
        "low_idle_ranges": low_idle_ranges,
        "low_idle_sample_count": low_idle_sample_count,
        "minimum_idle_percent": min(idle_percent),
        "maximum_cpu_percent": max(cpu_percent),
        "average_cpu_percent": (
            sum(cpu_percent) / len(cpu_percent)
        ),
    }


def plot_ram_usage(
    output_path: Path,
    elapsed_hours: list[float],
    ram_percent: list[float],
) -> dict[str, float]:
    """Plot RAM usage over the entire log."""
    smoothed_ram = moving_average(ram_percent, window=5)

    figure, axis = plt.subplots(figsize=(14, 6))

    axis.plot(
        elapsed_hours,
        smoothed_ram,
        label="RAM usage",
        linewidth=1.0,
    )

    axis.set_title("PX4 RAM usage over the entire log")
    axis.set_xlabel("Elapsed log time (hours)")
    axis.set_ylabel("RAM usage (%)")
    axis.set_ylim(0.0, 100.0)
    axis.grid(True, alpha=0.3)
    axis.legend(loc="best")

    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    plt.close(figure)

    return {
        "minimum_ram_percent": min(ram_percent),
        "maximum_ram_percent": max(ram_percent),
        "average_ram_percent": (
            sum(ram_percent) / len(ram_percent)
        ),
    }


def analyze_cpuload(
    ulog: Any,
    output_directory: Path,
    base_name: str,
) -> dict[str, Any]:
    """Extract, export, and plot the cpuload ULog topic."""
    dataset = get_dataset_or_none(ulog, "cpuload")

    if dataset is None:
        print("\nThe ULog does not contain a cpuload topic.")
        return {
            "available": False,
        }

    data = dataset.data

    if "timestamp" not in data or "load" not in data:
        print("\nThe cpuload topic is missing timestamp or load fields.")
        return {
            "available": False,
        }

    timestamps_us = [
        int(value)
        for value in data["timestamp"]
    ]

    cpu_percent = normalize_fraction_or_percent(data["load"])

    sample_count = min(len(timestamps_us), len(cpu_percent))

    timestamps_us = timestamps_us[:sample_count]
    cpu_percent = cpu_percent[:sample_count]

    if not timestamps_us:
        print("\nThe cpuload topic contains no samples.")
        return {
            "available": False,
        }

    first_timestamp = timestamps_us[0]

    elapsed_seconds = [
        (timestamp - first_timestamp) / 1_000_000.0
        for timestamp in timestamps_us
    ]

    elapsed_hours = [
        seconds / 3600.0
        for seconds in elapsed_seconds
    ]

    idle_percent = [
        max(0.0, min(100.0, 100.0 - cpu))
        for cpu in cpu_percent
    ]

    ram_percent: list[float] | None = None

    if "ram_usage" in data:
        ram_values = normalize_fraction_or_percent(data["ram_usage"])
        ram_percent = ram_values[:sample_count]

    csv_path = output_directory / f"{base_name}_cpuload.csv"
    cpu_plot_path = output_directory / f"{base_name}_cpu_usage.png"

    export_cpuload_csv(
        csv_path,
        timestamps_us,
        elapsed_seconds,
        cpu_percent,
        idle_percent,
        ram_percent,
    )

    dropout_markers = extract_dropout_markers(
        ulog,
        log_start_timestamp=first_timestamp,
    )

    cpu_summary = plot_cpu_usage(
        cpu_plot_path,
        elapsed_hours,
        cpu_percent,
        idle_percent,
        dropout_markers,
    )

    ram_plot_path: Path | None = None
    ram_summary: dict[str, float] | None = None

    if ram_percent is not None and ram_percent:
        ram_plot_path = output_directory / f"{base_name}_ram_usage.png"

        ram_summary = plot_ram_usage(
            ram_plot_path,
            elapsed_hours,
            ram_percent,
        )

    print_heading("CPU Time-Series Output", "-")
    print(f"Samples:              {sample_count}")
    print(f"Log duration:         {elapsed_hours[-1]:.3f} hours")
    print(
        f"Average CPU usage:    "
        f"{cpu_summary['average_cpu_percent']:.2f}%"
    )
    print(
        f"Maximum CPU usage:    "
        f"{cpu_summary['maximum_cpu_percent']:.2f}%"
    )
    print(
        f"Minimum CPU idle:     "
        f"{cpu_summary['minimum_idle_percent']:.2f}%"
    )
    print(
        f"Low-idle samples:     "
        f"{cpu_summary['low_idle_sample_count']}"
    )
    print(
        f"Low-idle ranges:      "
        f"{len(cpu_summary['low_idle_ranges'])}"
    )
    print(f"CPU plot:             {cpu_plot_path}")
    print(f"CPU CSV:              {csv_path}")

    if ram_plot_path and ram_summary:
        print(f"RAM plot:             {ram_plot_path}")
        print(
            f"Maximum RAM usage:    "
            f"{ram_summary['maximum_ram_percent']:.2f}%"
        )

    if getattr(ulog, "dropouts", []):
        print(
            f"ULog dropout records: "
            f"{len(getattr(ulog, 'dropouts', []))}"
        )

        if not dropout_markers:
            print(
                "Dropout durations were present, but this pyulog version "
                "did not expose their timestamps for plotting."
            )

    return {
        "available": True,
        "sample_count": sample_count,
        "duration_hours": elapsed_hours[-1],
        "cpu_plot_path": cpu_plot_path,
        "csv_path": csv_path,
        "ram_plot_path": ram_plot_path,
        "cpu_summary": cpu_summary,
        "ram_summary": ram_summary,
    }


def generate_watchdog_report(
    ulog: Any,
    output_path: Path,
    cpuload_result: dict[str, Any],
) -> None:
    """Write watchdog and time-series findings to a text report."""
    watchdog_top_lines = get_multiple_info_lines(
        ulog,
        "perf_top_watchdog",
    )

    watchdog_counter_lines = get_multiple_info_lines(
        ulog,
        "perf_counter_watchdog",
    )

    preflight_top_lines = get_multiple_info_lines(
        ulog,
        "perf_top_preflight",
    )

    watchdog_top = extract_top_summary(watchdog_top_lines)
    preflight_top = extract_top_summary(preflight_top_lines)

    counters = [
        parse_counter_line(line)
        for line in watchdog_counter_lines
    ]

    interesting_counters = [
        counter
        for counter in counters
        if is_interesting_counter(counter)
    ]

    with output_path.open("w", encoding="utf-8") as report:
        report.write("PX4 ULog Watchdog and CPU Analysis\n")
        report.write("=" * 60 + "\n\n")

        report.write("Watchdog snapshot\n")
        report.write("-" * 60 + "\n")
        report.write(
            f"perf_top_watchdog present: "
            f"{bool(watchdog_top_lines)}\n"
        )
        report.write(
            f"perf_counter_watchdog present: "
            f"{bool(watchdog_counter_lines)}\n\n"
        )

        if watchdog_top_lines:
            report.write(
                f"Task CPU:      {watchdog_top['task_cpu']}\n"
            )
            report.write(
                f"Scheduler CPU: {watchdog_top['scheduler_cpu']}\n"
            )
            report.write(
                f"CPU idle:      {watchdog_top['idle_cpu']}\n"
            )
            report.write(
                f"{watchdog_top['processes'] or ''}\n"
            )
            report.write(
                f"{watchdog_top['dma'] or ''}\n"
            )
            report.write(
                f"{watchdog_top['uptime'] or ''}\n\n"
            )

            report.write("Highest CPU tasks\n")
            report.write("-" * 60 + "\n")

            tasks = [
                task
                for task in watchdog_top["tasks"]
                if task["command"].lower() != "idle task"
            ]

            for task in sorted(
                tasks,
                key=lambda item: item["cpu_percent"],
                reverse=True,
            )[:20]:
                report.write(
                    f"{task['command']}: "
                    f"{task['cpu_percent']:.3f}% CPU, "
                    f"{task['stack_used']}/{task['stack_total']} stack "
                    f"({task['stack_percent']:.1f}%)\n"
                )

        if preflight_top_lines:
            report.write("\nPreflight CPU snapshot\n")
            report.write("-" * 60 + "\n")
            report.write(
                f"Task CPU:      {preflight_top['task_cpu']}\n"
            )
            report.write(
                f"Scheduler CPU: {preflight_top['scheduler_cpu']}\n"
            )
            report.write(
                f"CPU idle:      {preflight_top['idle_cpu']}\n"
            )

        report.write("\nCPU time-series\n")
        report.write("-" * 60 + "\n")

        if cpuload_result.get("available"):
            cpu_summary = cpuload_result["cpu_summary"]

            report.write(
                f"Samples: {cpuload_result['sample_count']}\n"
            )
            report.write(
                f"Duration: "
                f"{cpuload_result['duration_hours']:.3f} hours\n"
            )
            report.write(
                f"Average CPU: "
                f"{cpu_summary['average_cpu_percent']:.3f}%\n"
            )
            report.write(
                f"Maximum CPU: "
                f"{cpu_summary['maximum_cpu_percent']:.3f}%\n"
            )
            report.write(
                f"Minimum idle: "
                f"{cpu_summary['minimum_idle_percent']:.3f}%\n"
            )
            report.write(
                f"Low-idle threshold: "
                f"{LOW_IDLE_THRESHOLD_PERCENT:.3f}%\n"
            )
            report.write(
                f"Low-idle samples: "
                f"{cpu_summary['low_idle_sample_count']}\n"
            )
            report.write(
                f"Low-idle ranges: "
                f"{len(cpu_summary['low_idle_ranges'])}\n"
            )

            for index, time_range in enumerate(
                cpu_summary["low_idle_ranges"],
                start=1,
            ):
                report.write(
                    f"  Range {index}: "
                    f"{time_range[0]:.6f} to "
                    f"{time_range[1]:.6f} hours\n"
                )

            ram_summary = cpuload_result.get("ram_summary")

            if ram_summary:
                report.write(
                    f"Average RAM: "
                    f"{ram_summary['average_ram_percent']:.3f}%\n"
                )
                report.write(
                    f"Maximum RAM: "
                    f"{ram_summary['maximum_ram_percent']:.3f}%\n"
                )
        else:
            report.write("cpuload topic not available.\n")

        report.write("\nInteresting watchdog counters\n")
        report.write("-" * 60 + "\n")

        if interesting_counters:
            for counter in interesting_counters:
                report.write(
                    f"{counter['name']}:\n"
                    f"  {counter['details']}\n"
                )
        else:
            report.write(
                "No interesting watchdog counters were identified.\n"
            )

        report.write("\nLimitations\n")
        report.write("-" * 60 + "\n")
        report.write(
            "The cpuload topic contains total non-idle CPU load and RAM "
            "usage. It does not identify individual task CPU usage or "
            "separate scheduler CPU usage over time.\n"
        )
        report.write(
            "Individual task and scheduler values are only available in "
            "the perf_top watchdog/preflight snapshots.\n"
        )
        report.write(
            "logger_sd_write and logger_sd_fsync values in "
            "perf_counter_watchdog are cumulative performance-counter "
            "snapshots, not time-series datasets. They cannot be plotted "
            "over the full log unless the firmware logs a dedicated "
            "time-series topic for those values.\n"
        )


def print_watchdog_summary(ulog: Any) -> None:
    """Print a condensed watchdog summary to the console."""
    watchdog_top_lines = get_multiple_info_lines(
        ulog,
        "perf_top_watchdog",
    )

    watchdog_counter_lines = get_multiple_info_lines(
        ulog,
        "perf_counter_watchdog",
    )

    preflight_top_lines = get_multiple_info_lines(
        ulog,
        "perf_top_preflight",
    )

    print_heading("WATCHDOG SUMMARY")

    if not watchdog_top_lines and not watchdog_counter_lines:
        print("No watchdog performance snapshot was found.")
        return

    print("Watchdog snapshot present: Yes")
    print(
        "Its presence does not by itself prove that a watchdog reset occurred."
    )

    if watchdog_top_lines:
        watchdog_top = extract_top_summary(watchdog_top_lines)

        print_heading("Watchdog CPU Snapshot", "-")

        if watchdog_top["task_cpu"] is not None:
            print(
                f"Task CPU usage:      "
                f"{watchdog_top['task_cpu']:.2f}%"
            )

        if watchdog_top["scheduler_cpu"] is not None:
            print(
                f"Scheduler CPU usage: "
                f"{watchdog_top['scheduler_cpu']:.2f}%"
            )

        if watchdog_top["idle_cpu"] is not None:
            print(
                f"CPU idle:            "
                f"{watchdog_top['idle_cpu']:.2f}%"
            )

        tasks = [
            task
            for task in watchdog_top["tasks"]
            if task["command"].lower() != "idle task"
        ]

        if tasks:
            print_heading("Highest CPU Tasks", "-")
            print(
                f"{'PID':>6}  "
                f"{'CPU %':>7}  "
                f"{'Stack':>14}  "
                f"{'Stack %':>8}  "
                f"Command"
            )
            print("-" * 76)

            for task in sorted(
                tasks,
                key=lambda item: item["cpu_percent"],
                reverse=True,
            )[:15]:
                stack_text = (
                    f"{task['stack_used']}/{task['stack_total']}"
                )

                print(
                    f"{task['pid']:>6}  "
                    f"{task['cpu_percent']:>7.3f}  "
                    f"{stack_text:>14}  "
                    f"{task['stack_percent']:>7.1f}%  "
                    f"{task['command']}"
                )

    if watchdog_counter_lines:
        counters = [
            parse_counter_line(line)
            for line in watchdog_counter_lines
        ]

        interesting = [
            counter
            for counter in counters
            if is_interesting_counter(counter)
        ]

        print_heading("Interesting Watchdog Counters", "-")

        if interesting:
            for counter in interesting:
                print(
                    f"{counter['name']}:\n"
                    f"  {counter['details']}"
                )
        else:
            print("No interesting counters were identified.")

    if preflight_top_lines and watchdog_top_lines:
        preflight = extract_top_summary(preflight_top_lines)
        watchdog = extract_top_summary(watchdog_top_lines)

        print_heading("Preflight vs Watchdog CPU", "-")

        if preflight["idle_cpu"] is not None:
            print(
                f"Preflight CPU idle: "
                f"{preflight['idle_cpu']:.2f}%"
            )

        if watchdog["idle_cpu"] is not None:
            print(
                f"Watchdog CPU idle:  "
                f"{watchdog['idle_cpu']:.2f}%"
            )

        if (
            preflight["idle_cpu"] is not None
            and watchdog["idle_cpu"] is not None
        ):
            change = watchdog["idle_cpu"] - preflight["idle_cpu"]

            print(
                f"Idle change:        "
                f"{change:+.2f} percentage points"
            )


def main() -> None:
    print("=" * 72)
    print("PX4 ULog Information, Watchdog, CPU, and RAM Analyzer")
    print("=" * 72)

    try:
        ulg_path_text = select_ulg_file()

        if not ulg_path_text:
            print("\nNo ULog file was provided.")
            return

        ulg_path = Path(
            os.path.abspath(
                os.path.expanduser(ulg_path_text)
            )
        )

        if not ulg_path.is_file():
            raise FileNotFoundError(
                f"File does not exist:\n{ulg_path}"
            )

        if ulg_path.suffix.lower() != ".ulg":
            print(
                "\nWarning: selected file does not end in .ulg"
            )

        try:
            from pyulog import ULog
        except ImportError as exc:
            raise RuntimeError(
                "pyulog is not installed.\n\n"
                "Install dependencies using:\n"
                "    python -m pip install pyulog matplotlib"
            ) from exc

        output_directory = ulg_path.parent
        base_name = ulg_path.stem

        print(f"\nSelected ULog:\n{ulg_path}")
        print(f"\nOutput folder:\n{output_directory}")

        print("\nRunning ulog_info...\n")
        print("-" * 72)

        run_ulog_info(str(ulg_path))

        print("-" * 72)
        print("\nFinished standard ULog information output.")

        print("\nLoading ULog for detailed analysis...")
        ulog = ULog(str(ulg_path))

        print_watchdog_summary(ulog)

        cpuload_result = analyze_cpuload(
            ulog,
            output_directory,
            base_name,
        )

        report_path = (
            output_directory
            / f"{base_name}_watchdog_summary.txt"
        )

        generate_watchdog_report(
            ulog,
            report_path,
            cpuload_result,
        )

        print_heading("Generated Files", "-")
        print(f"Watchdog report: {report_path}")

        if cpuload_result.get("available"):
            print(
                f"CPU plot:        "
                f"{cpuload_result['cpu_plot_path']}"
            )
            print(
                f"CPU CSV:         "
                f"{cpuload_result['csv_path']}"
            )

            if cpuload_result.get("ram_plot_path"):
                print(
                    f"RAM plot:        "
                    f"{cpuload_result['ram_plot_path']}"
                )

        print_heading("Analysis Complete")

    except Exception:
        print("\nAn error occurred:\n")
        traceback.print_exc()

    finally:
        pause_before_exit()


if __name__ == "__main__":
    main()



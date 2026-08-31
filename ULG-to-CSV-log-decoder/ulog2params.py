#!/usr/bin/env python3
"""Export the initial parameters stored in a ULog file to a text file."""

import argparse
import os


def export_parameters(ulog_file_name, output_file=None, disable_str_exceptions=False):
    """Export parameters from *ulog_file_name* and return the output path."""
    from core import ULog

    ulog = ULog(ulog_file_name, disable_str_exceptions=disable_str_exceptions)

    if output_file is None:
        output_file = os.path.splitext(ulog_file_name)[0] + "_params.txt"

    with open(output_file, "w", encoding="utf-8") as param_file:
        for name, value in sorted(ulog.initial_parameters.items()):
            param_file.write(f"{name}={value}\n")

    return output_file


def main():
    """Command-line interface for exporting ULog parameters."""
    parser = argparse.ArgumentParser(
        description="Export the initial parameters in a ULog file to text"
    )
    parser.add_argument("filename", metavar="file.ulg", help="ULog input file")
    parser.add_argument(
        "-o", "--output", metavar="FILE", help="Output file (default: <input>_params.txt)"
    )
    parser.add_argument(
        "-i",
        "--ignore",
        action="store_true",
        help="Ignore string parsing exceptions",
    )
    args = parser.parse_args()

    output_file = export_parameters(args.filename, args.output, args.ignore)
    print(f"Writing {output_file}")


if __name__ == "__main__":
    main()

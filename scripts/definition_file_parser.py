#!/usr/bin/env python3

"""
Author:
    Lachlan Mares, lachlan.mares@adelaide.edu.au

License:
    ??

Description:
    ??
"""

from pathlib import Path


def parse_definitions_file(definitions_filepath: Path):
    def_dict = {}

    if definitions_filepath.is_file():
        definitions_file = open(definitions_filepath, "r")

        if definitions_file.mode == 'r':
            definitions = definitions_file.read().splitlines()

            definitions_file.close()
            dict_key = ''

            for line in [lines.split() for lines in definitions if len(lines.split()) > 0]:
                if line[0] == '//' and len(line) == 2:
                    dict_key = line[1]
                    def_dict[dict_key] = {}

                elif line[0] == '#define' and len(line) == 3:
                    if line[2].startswith('0x'):
                        def_dict[dict_key][line[1]] = int(line[2][2:], 16)
                    else:
                        def_dict[dict_key][line[1]] = int(line[2])

        else:
            raise Exception("File mode != r")

    else:
        raise Exception("Filepath is not a file")

    return def_dict


if __name__ == "__main__":
    project_dir = Path(__file__).resolve().parents[1]
    header_file = project_dir / 'arduino/engineering-team-motor/definitions.h'

    print(parse_definitions_file(header_file))
    
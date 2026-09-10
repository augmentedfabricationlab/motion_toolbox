"""Verify source hashes without importing or writing to the old repositories."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--workspace', type=Path, required=True)
    args = parser.parse_args()
    snapshot = json.loads((Path(__file__).resolve().parents[1]/'source_snapshot.json').read_text())
    changed = []
    count = 0
    for name, entry in snapshot.items():
        for relative, expected in entry['files'].items():
            path = args.workspace/name/relative.replace('\\', '/')
            if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
                changed.append(str(path))
            count += 1
    if changed:
        raise SystemExit('Source changes detected: ' + ', '.join(changed))
    print('{} original Python files match the initial SHA-256 snapshot.'.format(count))


if __name__ == '__main__':
    main()

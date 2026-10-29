#!/usr/bin/env python3
"""Copy Git-intended files, build from scratch and exercise the relocated workspace."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

GENERATED_DIRECTORIES = ('build', 'install', 'log')


def parse_arguments() -> argparse.Namespace:
    """Read the report path and optional official middleware binary prefix."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='/tmp/search_squad_clean_copy.json')
    parser.add_argument(
        '--dds-prefix',
        help='Optional extracted official binary prefix for restricted test hosts',
    )
    return parser.parse_args()


def copy_source_files(root: Path, destination: Path) -> list[str]:
    """Copy regular Git candidates with executable modes, excluding generated trees and symlinks."""
    git_listing = subprocess.check_output(
        ['git', 'ls-files', '--cached', '--others', '--exclude-standard', '-z'],
        cwd=root,
    )
    copied_files = []
    for name in sorted(set(git_listing.split(b'\0'))):
        if not name:
            continue
        relative_path = Path(os.fsdecode(name))
        assert relative_path.parts[0] not in (*GENERATED_DIRECTORIES, '.git'), relative_path
        source = root / relative_path
        assert source.is_file() and not source.is_symlink(), source
        target = destination / relative_path
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        copied_files.append(str(relative_path))

    assert not any((destination / name).exists() for name in GENERATED_DIRECTORIES)
    return copied_files


def clean_environment() -> dict[str, str]:
    """Use system Python and isolated ROS/Gazebo endpoints, without inherited overlays."""
    environment = {}
    for name in ('HOME', 'USER'):
        if name in os.environ:
            environment[name] = os.environ[name]
    environment.update(
        PATH='/usr/bin:/bin:/usr/sbin:/sbin',
        LANG='C.UTF-8',
        ROS_DOMAIN_ID='64',
        RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
        FASTDDS_BUILTIN_TRANSPORTS='UDPv4',
        GAZEBO_MASTER_URI='http://127.0.0.1:11464',
        ROS_LOG_DIR='/tmp/search_squad_clean_ros_logs',
        GAZEBO_LOG_PATH='/tmp/search_squad_clean_gazebo_logs',
    )
    return environment


def verification_shell(dds_prefix: Path | None) -> str:
    """Build before enabling temporary runtime binaries, then check hold and explorer_2."""
    dependency_check = 'rosdep check --from-paths src --ignore-src --rosdistro humble'
    build_command = (
        'colcon build --executor sequential --cmake-args '
        '-DCMAKE_BUILD_TYPE=RelWithDebInfo -DPython3_EXECUTABLE=/usr/bin/python3'
    )
    runtime_setup = ['source install/local_setup.bash']
    if dds_prefix is not None:
        dependency_check += ' --skip-keys rmw_cyclonedds_cpp'
        build_command = 'env -u RMW_IMPLEMENTATION ' + build_command
        runtime_setup.extend([
            'export AMENT_PREFIX_PATH="$SEARCH_SQUAD_DDS_PREFIX:$AMENT_PREFIX_PATH"',
            'export LD_LIBRARY_PATH="$SEARCH_SQUAD_DDS_PREFIX/lib:'
            '$SEARCH_SQUAD_DDS_PREFIX/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"',
        ])

    commands = [
        'source /opt/ros/humble/setup.bash',
        'set -e',
        dependency_check,
        'colcon list',
        build_command,
        *runtime_setup,
        'python3 tests/verify_runtime.py --mode baseline '
        '--output /tmp/search_squad_clean_baseline.json',
        'python3 tests/verify_runtime.py --mode demo --robot explorer_2 '
        '--output /tmp/search_squad_clean_demo_2.json',
    ]
    return '\n'.join(commands) + '\n'


def main() -> None:
    """Create the clean copy, execute its checks and report the actual child exit status."""
    args = parse_arguments()
    root = Path(__file__).resolve().parents[1]
    destination = Path(tempfile.mkdtemp(prefix='search-squad-clean-'))
    copied_files = copy_source_files(root, destination)
    environment = clean_environment()
    report = {
        'copy_path': str(destination),
        'source_files': copied_files,
        'count': len(copied_files),
    }

    dds_prefix = None
    if args.dds_prefix:
        dds_prefix = Path(args.dds_prefix).resolve()
        assert (dds_prefix / 'lib/librmw_cyclonedds_cpp.so').is_file()
        environment['SEARCH_SQUAD_DDS_PREFIX'] = str(dds_prefix)
        report['temporary_binary_dependency_prefix'] = str(dds_prefix)

    log_path = Path(args.output).with_suffix('.build.log')
    with log_path.open('w') as log:
        verification = subprocess.run(
            ['bash', '--noprofile', '--norc', '-c', verification_shell(dds_prefix)],
            cwd=destination,
            env=environment,
            stdout=log,
            stderr=subprocess.STDOUT,
        )
    report['build_and_runtime_log'] = str(log_path)
    report['result'] = 'passed' if verification.returncode == 0 else 'failed'
    Path(args.output).write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    raise SystemExit(verification.returncode)


if __name__ == '__main__':
    main()

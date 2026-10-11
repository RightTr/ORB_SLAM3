from pathlib import Path
from runpy import run_path


def generate_launch_description():
    make_launch = run_path(str(Path(__file__).with_name('d435i_common.py')))['make_launch_description']
    return make_launch('stereo')

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['lawnbot_description'],
    scripts=['/scripts/Driver.py',
             '/scripts/core/State.py',
             '/scripts/core/SearchNode.py'],
    package_dir={'': 'src'}
)

setup(**d)
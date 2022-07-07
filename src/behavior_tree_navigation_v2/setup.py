from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['behavior_tree_navigation_v2'],
    package_dir={'': 'scripts'}
)

setup(**d)

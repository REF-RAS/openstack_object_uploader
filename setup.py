from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# required packages
req_packages = [
]

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['uploader', 'tools'],
    package_dir={'': 'src'},
    python_requires='>=3.8',
    install_requires=req_packages
)

setup(**setup_args)

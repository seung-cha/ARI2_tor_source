from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


#Setup.py to allow imports from other packages in the same workspace.
_setup = generate_distutils_setup(
    packages=['ari_intent_consumer'],
    package_dir={'':'src'}
)

setup(**_setup)
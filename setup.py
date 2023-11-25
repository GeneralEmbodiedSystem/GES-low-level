from setuptools import find_packages
from distutils.core import setup

setup(
    name='AGI-low-level',
    version='1.0.0',
    author='thkkk',
    license="BSD-3-Clause",
    packages=find_packages(),
    author_email='bernard.hengk.tan@gmail.com',
    description='AGI-low-level',
    install_requires=['numpy', 'tensorboard', 'matplotlib', 'scipy', 'tqdm', ],
)

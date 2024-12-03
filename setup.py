from setuptools import setup, find_packages

setup(
    name='matchteris',
    version='1.0.0',
    packages=find_packages(),
    install_requires=[
        'mujoco',
        'gymnasium',
        'dm-control',
        'opencv-python',
        'numpy == 1.26',
    ],
)
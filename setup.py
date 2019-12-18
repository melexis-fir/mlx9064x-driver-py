from setuptools import setup
import sys
import platform

requires = ['pyserial>=3.4',
            'ctypes-callable>=1.0.0',
            ]

requirements_file = 'requirements.txt'
if platform.machine().startswith('armv'):
    requirements_file = 'requirements_rpi.txt'

with open(requirements_file) as f:
    requires += f.read().splitlines()

setup(
    name='mlx.mlx90640',
    version='1.0',
    description='Python library for MLX90640',
    license='Apache License, Version 2.0',
    entry_points = {'console_scripts': ['mlx90640-dump-frame = mlx.examples.mlx90640_dump_frame:main']},
    install_requires=requires,
    packages=['mlx/','mlx/pympt','mlx/examples'],
    classifiers=[
        # complete classifier list: http://pypi.python.org/pypi?%3Aaction=list_classifiers
        'Development Status :: 5 - Production/Stable',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Operating System :: Microsoft :: Windows',
        'Programming Language :: Python',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.4',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Topic :: Utilities',
    ],
)

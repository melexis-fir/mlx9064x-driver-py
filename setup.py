from setuptools import setup
import sys
import platform

version='1.3.0'

requires = ['pyserial>=3.4',
            'ctypes-callable>=1.0.0',
            'RPi.GPIO>=0.7.0 ; platform_machine=="armv7l"',
            'smbus2>=0.3.0; platform_machine=="armv7l"',
            'Jetson.GPIO>=2.0.8 ; platform_machine=="aarch64"',
            'smbus2>=0.3.0; platform_machine=="aarch64"'
            ]

with open("README.md", "r") as fh:
    long_description = fh.read()

setup(
    name='mlx9064x-driver',
    version=version,
    description='Python library for MLX90640-41',
    long_description=long_description,
    long_description_content_type="text/markdown",
    license='Apache License, Version 2.0',
    entry_points = {'console_scripts': ['mlx9064x-dump-frame = mlx.examples.mlx90640_dump_frame:main']},
    install_requires=requires,
    url = 'https://github.com/melexis-fir/mlx9064x-driver-py',   # Provide either the link to your github or to your website
    download_url = 'https://github.com/melexis-fir/mlx9064x-driver-py/archive/V'+version+'.tar.gz',
    packages=['mlx/','mlx/pympt','mlx/examples'],
    classifiers=[
        # complete classifier list: http://pypi.python.org/pypi?%3Aaction=list_classifiers
        'Development Status :: 5 - Production/Stable',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Operating System :: Microsoft :: Windows',
	'Operating System :: POSIX',
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

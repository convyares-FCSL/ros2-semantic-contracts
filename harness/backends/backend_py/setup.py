from setuptools import setup, find_packages

setup(
    name='backend_py',
    version='0.1.0',
    packages=find_packages(),
    entry_points={
        'console_scripts': [
            'backend_py=backend_py.__main__:main',
        ],
    },
    python_requires='>=3.10',
    install_requires=[
        # ROS 2 dependencies installed via apt (python3-rclpy, etc.)
        # Listed here for reference only
    ],
)

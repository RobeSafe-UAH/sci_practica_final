from glob import glob

from setuptools import find_packages, setup


setup(
    name='sci_mvsim',
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/sci_mvsim']),
        ('share/sci_mvsim', ['package.xml']),
        ('share/sci_mvsim/launch', glob('launch/*.launch.py')),
        ('share/sci_mvsim/definitions', glob('definitions/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SCI',
    maintainer_email='sci@example.com',
    description='Simulación SCI con MVSim, coche Ackermann, tráfico y sensores ROS 2.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['simulation_bridge = sci_mvsim.bridge:main']},
)

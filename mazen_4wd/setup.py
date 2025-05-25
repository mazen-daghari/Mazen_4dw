from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'mazen_4wd'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'opencv-python',
        'cv_bridge'
    ],
    zip_safe=True,
    maintainer='mazen_4wd Daghari',
    maintainer_email='dagmazen_4wd@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sphere_searcher = mazen_4wd.sphere_searcher:main',
        ],
    }
)
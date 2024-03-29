from setuptools import setup
import glob

package_name = 'air_lab_common'

setup(
        name=package_name,
        version='0.0.0',
        packages=[package_name],
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            ('share/' + package_name + "/screen", glob.glob('screen/*')),
            ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='morfr677',
        maintainer_email='mfroeh0@pm.me',
        description='TODO: Package description',
        license='TODO: License declaration',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                ],
            },
        )

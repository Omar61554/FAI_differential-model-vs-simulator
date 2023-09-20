from setuptools import find_packages, setup

package_name = 'G-model'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='omark',
    maintainer_email='wagih.omar11@gmai.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'coppelia = G-model.coppelia:main',
        'model = G-model.model:main',
        'location = G-model.location:main',
        'plot = G-model.plot:main',
        'plot2 = G-model.plot2:main',
        'simulation = G-model.simulation:main',
        'LocationSubscriber = G-model.LocationSubscriber:main',
        'Control = G-model.Control:main'
        

        ],
    },
)

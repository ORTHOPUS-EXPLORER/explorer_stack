from setuptools import find_packages, setup

package_name = 'pyvesc_explorer'

setup(
    name=package_name,
    version='0.0.0',
    packages=['pyvesc_explorer'] + find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [
            'resource/' + package_name
        ]),
        ('share/' + package_name, [
            'package.xml'
        ]),
        #('lib/'+ package_name,  [
            #package_name+'/__.py'
        #]),
    ],
    install_requires=['setuptools','pyvesc'],
    zip_safe=True,
    maintainer='BMi',
    maintainer_email='bmi.dev@redox.ws',
    description='pyVESC integration for Orthopus Explorer arm',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'pyvesc_explorer = pyvesc_explorer.pyvesc_explorer:main',
            'ros_explorer_bridge = pyvesc_explorer.ros_explorer_bridge:MainApp',
            'app_sim = pyvesc_explorer.app_sim:MainApp',
            #'app_scan = pyvesc_explorer.app_scan:MainApp',
        ],
    },
)

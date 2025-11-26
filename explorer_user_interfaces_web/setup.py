from setuptools import find_packages, setup

package_name = 'explorer_user_interfaces_web'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'requirements.txt']),
        ('share/' + package_name + '/launch', [
            'launch/web_gui.launch.py',
            'launch/explorer_with_web_gui.launch.py'
        ]),
        ('share/' + package_name + '/static', [
            'explorer_user_interfaces_web/static/style.css',
            'explorer_user_interfaces_web/static/app.js'
        ]),
        ('share/' + package_name + '/templates', [
            'explorer_user_interfaces_web/templates/index.html'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='isea.jean@orthopus.com',
    maintainer_email='isea.jean@orthopus.com',
    description='Web-based GUI for Explorer robot control interfaces',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_gui_node = explorer_user_interfaces_web.web_gui_node:main',
        ],
    },
)

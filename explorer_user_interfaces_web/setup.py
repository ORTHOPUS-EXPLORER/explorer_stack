from setuptools import find_packages, setup
from setuptools.command.develop import develop
from setuptools.command.install import install
import subprocess
import sys

package_name = 'explorer_user_interfaces_web'

# Dependencies that need pip
pip_dependencies = [
    'fastapi>=0.68.0',
    'uvicorn[standard]>=0.15.0',
    'websockets>=10.0',
    'jinja2>=3.0.0',
    'PyYAML>=5.4.0',
    'pynput',
]

def install_pip_dependencies():
    """Install pip dependencies to system/user site-packages"""
    try:
        # Install to user site-packages to avoid permission issues
        # and suppress script location warnings
        subprocess.check_call(
            [sys.executable, '-m', 'pip', 'install', '--user', '--no-warn-script-location'] + pip_dependencies,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE
        )
    except subprocess.CalledProcessError as e:
        # Fallback: try without --user flag
        subprocess.check_call(
            [sys.executable, '-m', 'pip', 'install', '--no-warn-script-location'] + pip_dependencies
        )

class PostDevelopCommand(develop):
    """Post-installation for development mode."""
    def run(self):
        develop.run(self)
        install_pip_dependencies()

class PostInstallCommand(install):
    """Post-installation for installation mode."""
    def run(self):
        install.run(self)
        install_pip_dependencies()

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/web_gui.launch.py',
        ]),
        ('share/' + package_name + '/static', [
            'explorer_user_interfaces_web/static/style.css',
            'explorer_user_interfaces_web/static/app.js'
        ]),
        ('share/' + package_name + '/templates', [
            'explorer_user_interfaces_web/templates/index.html'
        ]),
    ],
    install_requires=['setuptools'] + pip_dependencies,
    zip_safe=True,
    maintainer='thomas.solatges@orthopus.com',
    maintainer_email='thomas.solatges@orthopus.com',
    description='Web-based GUI for Explorer robot control interfaces',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_gui_node = explorer_user_interfaces_web.web_gui_node:main',
            'keyboard_to_joy = explorer_user_interfaces_web.keyboard_to_joy.keyboard_to_joy:main',
        ],
    },
    cmdclass={
        'develop': PostDevelopCommand,
        'install': PostInstallCommand,
    },
)

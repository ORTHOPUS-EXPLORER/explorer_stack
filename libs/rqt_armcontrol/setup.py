from setuptools import setup

package_name = 'rqt_armcontrol'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name + "/resource", ["resource/rqt_armcontrol.ui"]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["plugin.xml"]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='come',
    maintainer_email='come.butin@hotmail.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "rqt_armcontrol = rqt_armcontrol.rqt_armcontrol:main"
        ],
    },
)

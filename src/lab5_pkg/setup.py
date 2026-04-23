from setuptools import find_packages, setup

package_name = 'lab5_pkg'

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
    maintainer='alexandra.bacula',
    maintainer_email='alexandra.bacula@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "go_to_goal=lab5_pkg.go_to_goal_server:main",
            "go_to_goal_client=lab5_pkg.go_to_goal_client:main"
        ],
    },
)

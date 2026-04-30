from setuptools import find_packages, setup

package_name = 'newNav_pkg'

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
            "get_obs=newNav_pkg.get_obstacles:main",
            "go_to_goal=newNav_pkg.nav:main",
            "goal_client=newNav_pkg.goal_client:main",
            "pf_nav=newNav_pkg.nav_pf:main"
        ],
    },
)

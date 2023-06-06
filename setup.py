from setuptools import setup

package_name = 'ros2_term_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='heeyang',
    maintainer_email='huiyang1022@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "beagle_core = ros2_beagle.beagle_core:main", #비글의 core
            "beagle_sub_vel = ros2_beagle.beagle_sub_vel:main", #비글의 움직임만을 위한 노드
            "beagle_encoder_check = ros2_beagle.test_beagle:main",
            'move_beagle = move_beagle.move_beagle:main',
        ],
    },
)

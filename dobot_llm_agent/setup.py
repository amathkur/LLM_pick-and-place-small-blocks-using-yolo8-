from setuptools import find_packages, setup

package_name = 'dobot_llm_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'google-generativeai', 'opencv-python', 'torch', 'numpy'],
    zip_safe=True,
    maintainer='abdulhamid',
    maintainer_email='abdulhamid@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joint_state_publisher = dobot_llm_agent.joint_state_publisher:main',
            'llm_agent = dobot_llm_agent.llm_agent:main',
            'camera_publisher = dobot_llm_agent.camera_publisher:main',
            'robot_controller = dobot_llm_agent.robot_controller:main',
        ],
    },
)

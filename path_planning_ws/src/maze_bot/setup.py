from setuptools import setup
from glob import glob
import os

package_name = 'maze_bot'
package_name2 = 'PP_Algorithms'
package_name3 = 'Menus'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,package_name2,package_name3],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*')),
        (os.path.join('share', package_name,'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name,'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name,'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luqman',
    maintainer_email='noshluk2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'driving_node = maze_bot.DrivingNode:main' ,
            #'go_to_goal = maze_bot.GoToGoal:main',
            #'video_recorder = maze_bot.VideoSaver:main',
            'maze_solver = maze_bot.MazeSolver:main',
            'MyClock = maze_bot.Clock_Node:main'
        ],
    },
)

# ~/ros2_ws/src/mcvilloid/launch/mi_robot_launch.py
import os
import subprocess
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    webots_home = os.environ.get('WEBOTS_HOME')
    assert webots_home, "WEBOTS_HOME no está definido"

    # Candidatos: Windows y Linux
    candidates = [
        os.path.join(webots_home, 'webots.exe'),                                  # Windows típico
        os.path.join(webots_home, 'msys64', 'mingw64', 'bin', 'webots.exe'),      # Algunas instalaciones
        os.path.join(webots_home, 'webots-bin'),                                   # Linux
    ]
    webots_bin = next((p for p in candidates if os.path.exists(p)), None)
    assert webots_bin, f"No encontré Webots en: {candidates}"

    world_unix = '/home/maldonado/ros2_ws/src/mcvilloid/worlds/Pm01.wbt'
    world_arg = world_unix

    # Si es el .exe de Windows, convertir ruta del mundo a formato Windows
    if webots_bin.lower().endswith('.exe'):
        world_arg = subprocess.check_output(['wslpath', '-w', world_unix], text=True).strip()

    return LaunchDescription([
        ExecuteProcess(
            cmd=[webots_bin, '--stdout', '--stderr', world_arg],
            output='screen'
        )
    ])

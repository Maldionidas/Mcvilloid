# ~/ros2_ws/src/mcvilloid/launch/mi_robot_launch.py
"""
Launch de Webots desde ROS 2 para el proyecto Mcvilloid.
-------------------------------------------------------

Este launch:

- Lee la variable de entorno `WEBOTS_HOME` para localizar el binario de Webots.
- Soporta instalaciones típicas en:
    * Windows (`webots.exe`, o dentro de `msys64/mingw64/bin/webots.exe`)
    * Linux (`webots-bin`)
- Selecciona un mundo fijo (`sample2.wbt`) dentro del workspace.
- Si el binario encontrado es `.exe` (Webots en Windows desde WSL),
  convierte la ruta del mundo a formato Windows usando `wslpath -w`.
- Lanza Webots con:
    * `--stdout` y `--stderr` para redirigir la salida a la consola de ROS 2.

Dependencias:
- `WEBOTS_HOME` debe estar definido en el entorno.
- En el caso de usar Webots en Windows a través de WSL, se requiere `wslpath`.
"""

import os
import subprocess

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description() -> LaunchDescription:
    """
    Genera la LaunchDescription para arrancar Webots con el mundo deseado.

    Flujo:
    1. Obtiene `WEBOTS_HOME` desde el entorno.
    2. Construye una lista de candidatos posibles para el binario de Webots.
    3. Selecciona el primero que exista en el sistema de archivos.
    4. Define la ruta del mundo `.wbt` (actualmente fija a `sample2.wbt`).
    5. Si el binario es `.exe` (Windows), convierte la ruta del mundo a formato Windows.
    6. Devuelve una LaunchDescription con un `ExecuteProcess` que llama a Webots.
    """
    webots_home = os.environ.get("WEBOTS_HOME")
    assert webots_home, "WEBOTS_HOME no está definido"

    # Candidatos típicos de instalación: Windows y Linux
    candidates = [
        os.path.join(webots_home, "webots.exe"),                             # Windows típico
        os.path.join(webots_home, "msys64", "mingw64", "bin", "webots.exe"), # Algunas instalaciones en Windows
        os.path.join(webots_home, "webots-bin"),                             # Linux
    ]

    # Seleccionar el primer binario existente
    webots_bin = next((p for p in candidates if os.path.exists(p)), None)
    assert webots_bin, f"No encontré Webots en: {candidates}"

    # Ruta del mundo en formato Linux/WSL (ajustada a tu workspace actual)
    world_unix = "/home/maldonado/ros2_ws/src/mcvilloid/worlds/sample2.wbt"
    world_arg = world_unix

    # Si el binario es .exe (Webots en Windows), convertir la ruta a formato Windows vía wslpath
    if webots_bin.lower().endswith(".exe"):
        world_arg = subprocess.check_output(
            ["wslpath", "-w", world_unix],
            text=True,
        ).strip()

    # Lanzar Webots como proceso externo
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[webots_bin, "--stdout", "--stderr", world_arg],
                output="screen",
            )
        ]
    )

from setuptools import setup

package_name = 'sensors'  # Utilisez le même nom que le package C++

setup(
    name=package_name,
    version='0.0.0',
    packages=['src'],  # Le répertoire contenant le code Python
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'camera_processing_py_node = src.camera_processing_py_node:main',  # Déclare l'exécutable pour le node Python
        ],
    },
)

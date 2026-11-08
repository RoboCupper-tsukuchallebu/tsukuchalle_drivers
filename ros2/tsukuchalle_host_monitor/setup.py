from setuptools import setup

PACKAGE_NAME = "tsukuchalle_host_monitor"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=[PACKAGE_NAME],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/config", ["config/host_monitor.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [f"host_monitor_node = {PACKAGE_NAME}.host_monitor_node:main"],
    },
)

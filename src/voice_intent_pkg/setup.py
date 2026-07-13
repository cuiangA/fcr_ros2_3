from setuptools import find_packages, setup


package_name = "voice_intent_pkg"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="FCR Dev Team",
    maintainer_email="dev@fcr-project.org",
    description="Console-driven BERT intent inference for the FCR voice control chain.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "bert_console_voice_node = "
            "voice_intent_pkg.bert_console_voice_node:main",
        ],
    },
)

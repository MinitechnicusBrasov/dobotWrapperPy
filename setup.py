# mypy: disable-error-code="import"
import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="dobotWrapperPy",
    packages=["dobotWrapperPy", "dobotWrapperPy.enums"],
    package_data={"dobotWrapperPy": ["py.typed"], "dobotWrapperPy.enums": ["py.typed"]},
    include_package_data=True,
    version="0.1.7",
    description="Python library for Dobot Magician For Minitechnicus Courses",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="Minitechnicus",
    author_email="dev@minitechnicus.org",
    url="https://github.com/andreishark/dobotWrapperPy",
    download_url="https://github.com/andreishark/dobotWrapperPy",
    keywords=["dobot", "magician", "robotics", "minitechnicus"],
    classifiers=[],
    install_requires=["pyserial==3.4"],
)

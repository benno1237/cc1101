from distutils.core import setup

setup(
    name="cc1101",
    packages=["cc1101"],
    version="0.0.1a1",
    license="MIT",
    description="CC1101 driver for Raspberry Pi",
    author="benno1237",
    author_email="benno.kollreider@gmail.com",
    url="https://github.com/benno1237/cc1101",
    download_url="https://github.com/benno1237/cc1101/archive/refs/tags/0.0.1.tar.gz",
    keywords=["CC1101"],
    install_requires=[
        "pigpio",
        "bitstring",
    ],
    extras_require={
        "docs": [
            "sphinx",
            "sphinx-rtd-theme",
            "sphinx-prompt",
            "autodocsumm"
        ]
    },
    classifiers=[
    'Development Status :: 3 - Alpha',
    'Intended Audience :: Developers',
    'Topic :: Software Development :: Build Tools',
    'License :: OSI Approved :: MIT License',
    'Programming Language :: Python :: 3.9',
    'Programming Language :: Python :: 3.10',
    ],
)
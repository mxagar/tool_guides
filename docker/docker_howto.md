# Docker Howto

This file was created while following the Udemy course by Bret Fisher
Docker Mastery: with Kubernetes +Swarm from a Docker Captain.

Mikel Sagardia, 2021.
No warranties.

## Section 1: Introduction

### What is Docker? Why Docker?

Created in 2013 as open source project, now maintained by Docker Inc.

Paradigm shifts have happened in the past, requiring migrations:
- 90's: shift to PC networks
- 00's: virtualization
- 10's: Cloud
- Now: we are going from hosts to containers

Why docker?
- Like always, it's about speed in all the steps of the lifecycle.
- Containers reduce complexity: we package our code in a container and it runs everywhere
- Applications usually don't need to be changed to be used on docker containers, but they need to be packaged

### Resources


Original repo in

https://github.com/bretfisher/udemy-docker-mastery

Forked to

https://github.com/mxagar/udemy-docker-mastery

Additional material in my Dropbox:

- Cheat Sheet
- Slides
- Commads TXT files

See also the **FAQ** section.

## Section 2: Setting Up: Installation & Co.

### 2.1 Versions / Editions

- Each operating system has its editions and versions
- There is the Community Edition (CE, free), and the Enterprise Edition (EE, not free)
	- The EE has extra products and support, and it is certified
	- CE edition is for free and has Stable vs Edge versions
		- Edge is beta and comes out every month, supported for a smonth
		- Stable comes once per quarter (that's the recommended usually), suppported for 4 months
		- What prevviously was called 'Docker Engine' is now 'Docker CE'
		- We can extend the support period with EE
- The version matters because the industry is advancing very fast!
- Three types of installations
	1. Direct local: Linux only; each kernel and architecture has its version/edition, do not use default generic packages!
	2. Mac/Windows suite: they do not support Docker natively, but need to start a small virtual machine; however, we get a suite of tools and a GUI for interacting with Docker
	3. Cloud: AWS/Azure/Google; these are usually teh Linux version but with extended features for cloud platforms
- Since 2017 and version `1.13`, the `YY.MM` versioning convention is used, like with Ubuntu: `17.03`, `17.04`, ...


### 2.2 Installation

#### Windows / Mac: General notes

- Download from https://www.docker.com/products/docker-desktop
	- Preferably use the Stable version
- Download installer and install docker
- Assuming Windows 10 Pro; Windows Home Editions need extra installations and use the Docker Toolbox
- The Windows version
	- requires Hyper-V, a native virtual machine platform for Windows
	- requires Powershell
	- has specific **Windows containers** if the newest editions are used; all other containers are Linux containers
- The Mac version
	- works on Yosemite, so even in very old Macs; if older, we need Docker Toolbox
	- don't use the brew install, because it's only the CLI

#### Mac installation and setup steps

- Install img
- Menu icon appears
- Menu > Preferences
	- Resources: File sharing: Shared volumes should appear all here
	- Resources: Advanced: we see the specs of the native virtual machine running docker; note that resources are not used unless required
- Use the terminal, and check docker is running: `docker version`
- Install docker-machine with the commad line in: https://docs.docker.com/machine/install-machine/
- Command+TAB completion installation (really cool util that shows options with TABx2): https://docs.docker.com/docker-for-mac/#install-shell-completion
```bash
	# Install bash completion utility with brew
	brew install bash-completion
	# Link files
	etc=/Applications/Docker.app/Contents/Resources/etc
	ln -s $etc/docker.bash-completion $(brew --prefix)/etc/bash_completion.d/docker
	ln -s $etc/docker-compose.bash-completion $(brew --prefix)/etc/bash_completion.d/docker-compose
	# Add to ~/.bash_profile
	[ -f /usr/local/etc/bash_completion ] && . /usr/local/etc/bash_completion
	# Load again 
	source ~/.bash_profile
```

#### Linux

- Docker was made for Linux, no emulation is done under the hood, as with Windows or Mac
- Docker works for most distributions, but each needs a spcefic edition
- Do not use the default packages with `apt`, or any other pre-installed setups!
- Instead, go to the docker page and follow th einstructions, e.g., for Ubuntu
- Another option: run (Edge will be installed)
	`curl -sSL https://get.docker.com/ | sh`
- Add the user to the docker group to avoid doing sudo every time (but that is not allowed in all Linux versions)
	`sudo usermod -aG docker mxagar`
- For Linux, Docker Machine and Docker Compose need to be installed additionally, get the command lined from
	- https://docs.docker.com/compose/install/
	- https://docs.docker.com/machine/install-machine/
	- Note that docker machine and compose need to be updated manually and regularly

#### Resources

- Windows: cmder is a nice cmd prompt: https://cmder.net/
- Mac: Bret uses iTerm2
- **Visual Studio Code is recommended**: Docker plugins are available, **install them**!
- Github repo: clone it (I forked it): https://github.com/BretFisher/udemy-docker-mastery
- Docker Desktop for Mac: Manual: https://docs.docker.com/docker-for-mac/

## Section 3: Creating and Using Containers

- Containers are the building blocks of docker

```bash

docker version

```

## Section 4: Contanier Images


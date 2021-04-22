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

### Versions / Editions

- Each operating system has its editions and versions
- There is the Community Edition (CE, free), and the Enterprise Edition (EE, not free)
	- The EE has extra products and support, and it is certified
	- CE edition is for free and has Stable vs Edge versions
		- Edge is beta and comes out every month
		- Stable comes onece per quarter
- The version matters because the industry is advancing very fast!
- Three types of installations
	1. Direct local: Linux only; each kernel and architecture has its version/edition, do not use default generic packages!
	2. Mac/Windows suite: they do not support Docker natively, but need to start a small virtual machine; however, we get a suite of tools and a GUI for interacting with Docker
	3. Cloud: AWS/Azure/Google; these are usually teh Linux version but with extended features for cloud platforms

### Installation

Windows / Mac

	https://www.docker.com/products/docker-desktop
	Download installer and install docker
	Assuming Windows 10 Pro; Windows Home Editions need extra installations and use the Docker Toolbox

Linux

	Do not use the default packages with `apt`
	run
		`curl -sSL https://get.docker.com/ | sh`
	
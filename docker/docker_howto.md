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
- **IMPORTANT NOTE**: always use TAB autocompletion and `--help` as we type commands!


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
- Example: check docker is running by querying its version

```bash
# Version information, check if docker is installed and runnig
docker version

# A lot of config info
docker info

# Get all possible commands
# Some of them are management commands that accept a subcommand
# These were introduced later as the number of commands increased
# docker <command> <subcommand>
# Example: 
# (old, but backward compatible) docker run
# (new style) docker container run
docker
```

### Running Containers

- Images vs. containers
	- The image is the collection of binaries and libraries of our application
	- The container is a running instance of the image: you can have many containers running the same image
- Registries: as Github is a registry for code, the registry of docker images is Docker Hub: [hub.docker.com](hub.docker.com)
	- Official SW providers create and maintain their images, which can be downoaded/pulled and started locally as docker containers
	- Examples: nginx, ubuntu, python, golang, node, mongodb, postgres, ...
- We run a container with `docker container run` and the following happens
	- Docker looks for the image locally, if not found, it downloads it from Docker Hub; it is stored in the image cache
	- Latest version is download by default, but we can specify concrete versions
	- When a container is run, a new layer of changes is generated on the image, the image is not copied
	- A running container has a virtual IP on a private network and we can forward data to its ports
	- We can define a `CMD` in the image and pass the command when starting/running the container
- Example: Running an nginx server container:

```bash
# Docker looks for an image called nginx
# nginx is a web server
# Docker pulls the nginx image from Docker Hub
# and it starts it as a new process/container
# It opens the port 80 the host IP and sends all traffic through it
# to the port 80 of our container
# if host port 80 is taken, we could try 8888:80
docker container run --publish 80:80 nginx
# Open browser and go to localhost (or localhost:8888)
# nginx server notification appears

# Run in the background
# We get the container id back, to stop it with docker stop
docker container run --publish 80:80 --detach nginx

# If we open VS Code, containers & images appear in the tab

# List running containers shown, with id
docker container ls
docker ps

# Stop with Ctrl+C, or if detached
# docker stop <container id/name>
# BUT: we only stop the container, not remove it
# Note: using the first 3 letters of the id is enough
docker stop fecc962e3e6e
docker container stop fecc962e3e6e

# Show all containers that run at some point: running now and exited
# We get a table
# CONTAINER_ID, IMAGE, COMMAND, CREATED, STATUS, PORTS, NAMES
# Names are automatic (hacker & cypher names)
# unless we choose one with --name
docker container ls -a

# Start a nginx server container with name webhost
docker container run --publish 80:80 --detach --name webhost nginx

# Use container to generate some logs, e.g.,
# with nginx refresh the browser
# Display logs of container with name webhost
docker container logs webhost

# See top processes running in container named webhost
docker container top webhost

# See all subcommands commands for container
docker container --help

# Remove containers with their id; they won't appear with ls -a
# If running, we need to stop and rm, or force with rm -f
docker container ls -a
docker container rm 8db fec d76 880 889
docker container rm -f 3fd
```

- Containers are not really virtual machines, they are in fact just processes!
	- On Mac, a mini virtual machine is started and the container run in there
	- But on Linux, even containers are processes running on the host, not inside a virtual machine
	- Windows containers are different to the other (linux) containers: since 2017, Windows containers are EXEs running on the Windows kernel
	- Related links
		- [https://www.youtube.com/watch?v=sK5i-N34im8&list=PLBmVKD7o3L8v7Kl_XXh3KaJl9Qw2lyuFl](https://www.youtube.com/watch?v=sK5i-N34im8&list=PLBmVKD7o3L8v7Kl_XXh3KaJl9Qw2lyuFl)
		- [https://github.com/mikegcoleman/docker101/blob/master/Docker_eBook_Jan_2017.pdf](https://github.com/mikegcoleman/docker101/blob/master/Docker_eBook_Jan_2017.pdf)
		- [https://www.bretfisher.com/docker-for-mac-commands-for-getting-into-local-docker-vm/](https://www.bretfisher.com/docker-for-mac-commands-for-getting-into-local-docker-vm/)
		- [https://www.bretfisher.com/getting-a-shell-in-the-docker-for-windows-vm/](https://www.bretfisher.com/getting-a-shell-in-the-docker-for-windows-vm/)

```bash
# Start a MongoDB
# -d = --detach
docker run --name mongo -d mongo

# See the mongo container process: mongod
docker container top mongo

# On Linux, we'd see that mongod is a process on th ehost itself
ps aux | grep mongo

# If we stop and rm the container
# No mongod process will appear with ls -a nor with ps aux
docker container rm -f mongo
```

Exercise: managing several containers with env variables and looking for log output
```bash
# Start 3 containers with their typical ports open: nginx, https, mysql
# If images not available, downloaded from registries
# MySQL: we pass an environment variable with --env = -e
docker container run -d --name nginx --publish 80:80 nginx
docker container run -d --name mysql --publish 3306:3306 --env MYSQL_RANDOM_ROOT_PASSWORD=yes mysql
docker container run -d --name httpd --publish 8080:80 httpd

# For MySQL, we need to copy the random password generated to log in to it
# We must look at the logs for that
# ... "GENERATED ROOT PASSWORD: eilei1shohngohYaig4CohNohpouta1o"
docker container logs myql

# Stop and remove containers; check they are removed
# We can use the name or the ID, which is obtained also tabbing after stop
docker container stop mysql nginx https
docker container rm mysql nginx https
docker container ls -a
```

### CLI Process Monitoring

It is useful to observe the properties and performance of containers from a our host.

```bash
# Start some containers
docker container run -d --name nginx nginx
docker container run -d --name mysql --env MYSQL_RANDOM_ROOT_PASSWORD=yes mysql

# For MySQL, if pw needed, we can check the output
docker container logs mysql

# Check the processes within the containers
docker container top nginx

# Inpect: JSON with information about how the container was started
# VERY IMPORTANT!
docker container inspect mysql

# Show live performance of all running containers
# CPU, mem usage, networking, etc.
docker container stats

# Show live performance of mysql container
docker container stats mysql
```

### Getting a shell inside a container

It is not necessary doing `ssh` to a container to get a shell within it.
That can be done with the options `-it`, `-ai` and `exec`.
Note that once inside, we can install software on the container.
However, if the container is exited, unless with `start` it again, a new container of the same image won't have the installed software.

```bash
# Start a container and get a shell to it with -it
# -t: pseudo TTY, a pseudo shell
# -i: keep session of TTY open
# bash: optional command passed to the container; in our case, bash shell
# the container is nginx, but its name is proxy
# we log in as root to our container, we can use the shell there as usual!
# root@container_id
docker container run --name proxy -it nginx bash
# For exiting: exit; BUT: the container stops!
exit

# Same with a Ubuntu container: run it with a shell
# A light version of ubuntu is installed if not available
docker container run --name ubuntu -it ubuntu bash

# The Ubuntu version is so light that it has nothing
# We can install curl on it using the package manager
apt-get update
apt-get install -y curl
# We can use curl now!
curl google.com
# Exit: it stops the container
exit

# If we want to re-run our Ubuntu again use
# `start -ai` instead of `run -it`
# that way, the previous container with installed stuff is started
# otherwise, with run, we start a new container without installed things
docker container start -ai ubuntu
curl google.com

# If we want to open a shell of a running container that was lauched without -it
# we do `exec -it` + container_name + command (=bash)
docker container exec -it proxy bash
exit
# Note that now, conatiner is not stopped when we exit
# because an extra process for the shell was created with exec
```

If we want to use a super-light Linux distro, we need Alpine; but it has no `bash` installed.

```bash
# We pull the linux image of the smallest Linux distro: Alpine
docker pull alpine
# Alpine image is smaller than 6MB
docker image ls
# Alpine does not have bash installed, but sh
docker container run --name alpine -it alpine sh
# The package manager of Alpine is apk; we can install bash with it to use it later
apk
exit
```

### Docker Networks: Private and Public Communications

Docker is designed under the motto *batteries included, but removable*; that means we have already many features running out-of-the-box, but we can still change them at our will.

Docker container communication management is very flexible.
Some important communication/networking concepts:

- We can create virtual networks with groups of containers; the default virtual network is called `docker0` or `bridge`,  which appears if we do `ifconfig`
- We have a firewall protecting communications (in/out) with the internet and also between virtual networks 
- Containers within a virtual network can communicate between each other freely, no port forward needs to be done
- If we want to allow communication between a container and the internet or with a container from another virtual network on the same hoset, we need to forward ports; then, the communication would go through the firewall
- Only one container can be forwarded to a host port!

```bash
# Create an nginx container with port 80 forwarded: -p == --publish
# host port 80 traffic forwarded to -> container port 80
# We also say port 80 of the host was exposed
docker container run -p 80:80 --name webhost -d nginx

# Check the forwarded ports (exposed) of our container
docker container port webhost

# The IP address of the container is not necessary that of the host
# Check it with inspect
docker container inspect webhost | grep IP

# We can also use --format if we know the name of the JSON node
docker container inspect --format '{{ .NetworkSettings.IPAddress }}' webhost
# and we get sth like 172.17.0.2
# while our host IP is
ifconfig en0
# sth like 192.168.179.12
```

More on the `--format` option: [https://docs.docker.com/config/formatting/](https://docs.docker.com/config/formatting/).

Note: **nginx has no `ping` anymore: either install it with `apt-get` or use `nginx:alpine` instead of `nginx`**.

### Docker Networks: CLI Management of Virtual Networks

How networks work (as far as I understand):

- We have the internet and local networks, which can be divided into subnets inside the local network.
- Every device conneted to the internet has 2 IP addresses, or, in other words, its complete IP address has two identifiers: the host identifier and the network identifier. The network identifier is the IP visible to the rest of the Internet, the host iddentifier is the IP visible to the local network.
- An IP address has 32 bits divided in 4 octets of bits, like `X.X.X.X`, with `X=[0,255]`.
- The gateway device (router) connects local devices to other networks: every device in a local network talks to the gateway which forwards packets back and forth.
- A subnet mask is used to divide the current network in subnets, or in other words, we further branch the local host IP with addditional *surnames*. A mask of the type `X.X.X.X` is assigned to the subnet with `X = {0,255}`, depending on the number of devices in the subnet. The value `255` means *allocated for the network prefix*, while the value `0` means *free to subdivide for host addressing*; thus, if the first three octets are `255` and the last `0`, i.e., `255.255.255.0`, we'd have 24 bits for the network and 8 bits for further devices, that is 256 ddevices.
- However, I think for brevity, the subnet mask is given with the IP address followed by the number of network bits in the mask: `10.0.1.1/24`, which really means IP `10.0.1.1` and subnet mask `255.255.255.0`
- NIC = Network interface controller = network card = physical IP; when creating conatiners and virtual networks, we create virtual NICs, and connect and disconnect them.

For more, see: [https://avinetworks.com/glossary/subnet-mask/](https://avinetworks.com/glossary/subnet-mask/)


```bash
# Show networks
# usually, we see 3 networks:
# - bridge = docker0: default bridge connecting our docker containers to our physical host network; all our containers are by default attached to it 
# - host: special network which attaches the container to the host interface wiithout the bridge; there are pros & cons: no protection, but sometimes better performance for high throughput networking
# - none: like having an interface which is not attached to anything, it removes eth0 and leaves the container with localhost
docker network ls

# Inspect a network: we get a JSON with many infos:
# containers attached to it, etc.
# Dafault subnet mask is usually 172.17.0.0/16, gateway IP: 172.17.0.1
# Subnet mask:  
# Gateway IP: 
docker network inspect <network name, eg: bridge>

# Create a new virtual network called my_app_net
docker network create my_app_net
# We can check our new network
# The default drive is bridge: functionalities taken from bridge
docker network ls
# We could also specify the driver we'd like to use, and many other options, see with --help
docker network create --help

# We can specify a network when creating a container
docker container run -d --name new_nginx --network my_app_net nginx
# We can check that with
docker network inspect my_app_net

# But we can also attach/connect a network to an existing container
docker network connect --help
docker network connect [options] <network> <container>
docker network connect my_app_net webhost
# We see my_app_net has two conatiners attached: new_nginx, webhost
docker network inspect my_app_net
# We see webbhost is connected to two networks: bridge, my_app_net
docker conatiner inspect webhost

# Detach a network from container
docker network disconnect [options] <network> <container>
```

### Docker Networks: DNS and How Containers Find Each Other


## Section 4: Container Images


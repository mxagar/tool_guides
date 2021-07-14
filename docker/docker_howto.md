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
	2. Mac/Windows suite: they do not support Docker natively, but need to start a small linux virtual machine; however, we get a suite of tools and a GUI for interacting with Docker
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

- Virtual IP addresses of our container will change, so we need to use DNS naming to allow comms between them reliably
- The name of the container is its DNS!
- BUT: the default `bridge` network does not do it automatically, so we have two options
	1. Create our custom networks and attch containers to them (recommended); custom networks create DNS servers with container names
	2. Use the `--link list` option whe creating a container

Example: `ping` from a container to another:
```bash
# We start two containers and set them in the same custom network
docker container run -d --name webhost1 nginx
docker container run -d --name webhost2 nginx
docker network connect my_app_net webhost1
docker network connect my_app_net webhost2
# Install ping in a container
docker container exec -it webhost1 /bin/bash
apt-get update
apt-get install inetutils-ping
exit
# Ping from the container with ping to the other using the container name (DNS) 
docker container exec -it webhost1 ping webhost2
```

Assignment 1: check `curl --version` in 2 different Linux distributions
```bash
# Run CentOS 7
# Option --rm: automatically remove container when it exits
docker container run --rm -it centos:7 bash
# In CentOS
yum update curl
curl --version
exit
# Run Ubuntu from another shell
docker container run --rm -it ubuntu:14.04 bash
# In Ubuntu
apt-get update && apt-get install -y curl
curl --version
exit
# Since we used --rm, now no containers should be running/paused
docker container ls -a
```

Assignment 2: DNS Round Robin Test.
*Round Robin* is a load balancing technique which consists in distributing resources cicularly as requests come. For instance, if we have 10 kernels and launch 20 threads, we would start running threads 1, 2, 3, ... in kernels 1, 2, 3, ... and then threads 11, 12, 13 in kernels 1, 2, 3... again.
*Round Robin DNS*, analogously, consists in having several servers and their respetcive IPs behind a DNS name. For instance, the DNS google.com is extenced to have several IP adresses and servers behind, which are assigned in ROund Robin order to requests from the internet as they arise.
```bash
# We can assign the same alias DNS name to multiple containers 
# Create a network called dude
# --network-alias = --net-alias
docker network create dude
# We launch two containers of th eimage elasticsearch:2
# We assign the alias name 'search'
docker container run -d --net dude --network-alias search elasticsearch:2
docker container run -d --net dude --network-alias search elasticsearch:2
# Check the container running: 2x elasticsearch
# Note that the port tcp/9200 is open
docker container ls -a
# We can now start alpine/centos and look for DNS names
# with --rm the container is remooved after executing
# the image elasticsearch is especially useful for the example
# because it delivers a JSON with container info in port 9200 when requested,
# eg, with centos.
# As wee query the container ID or name changes between the 1st alasticsearch and the 2nd
# even though we query the same DNS name. That's Round Robin DNS balancing
docker container run --rm --net dude alpine nslookup search
docker container run --rm --net dude centos curl -s search:9200
docker container run --rm --net dude centos curl -s search:9200
docker container run --rm --net dude centos curl -s search:9200
...
# Stop elasticsearch containers
docker container stop 0e3 319
docker container stop rm 319
docker container ls -a
```

## Section 4: Container Images

An image is basically (1) the application binaries and dependencies (2) and the metadata on how to run it.

Images do not have a complete OS, no kernel or kernel modules (= drivers) either! Usually they are of a similar size as the application. However, we could also have a very big image, for instance, when the container has a whole distribution of Ubuntu.

Some basic commands:
```bash
# Check the images we have pulled/downloaded
docker image ls
# Download/pull an image, latest tag
docker pull nginx
# Download/pull an image, specific version and tag
docker pull nginx:1.19.10-alpine
```
###  Docker Hub

Sign up at [Docker Hub](hub.docker.com).
We can have our images in our account.
Explore which containers are available, eg, nginx.

Analyzing images:
- There is always one official one, with a simple image name; the others follow the template `org_name/image-property`, eg: `centos/nginx-112-centos7`.
- Look always at the number of pulls/downloads and stars.
- Official images are a great way to start, look at their documentation; [list of official images](https://github.com/docker-library/official-images/tree/master/library).
- Then, usually, we take an official image and modify it and save it as our own image; when can later upload it to docker hub, in a similar fashion as with Github.
- Look at the version tags of the official images; usually we download the tag `latest`, if nothing specified. However, the best practice is to track the exact version and pull it specifically.
- Note that a version can have several tags, eg: `latest`, `1.19.10`, `mainline`, `1`, `1.19`; however, all point to version `1.19.10`. We can see that by pull them all and then checking the image id with `docker image ls`: they all have the same SHA or id.

### Image Layers

Images are designed using the union file system concept, making layers around the changes.
The main idea is that we have a basis layer (eg., Ubuntu) and we add layers of changes to it: add a server, open a port, install something, etc. Each command or change is a layer and the sum of all layers together with the initual set of files (ie., Ubuntu) is the image. The key is that we store the layers, not the image as a monolith; that way, we can easily make branches from images, and the remain compact.

Containers are basically a running copy and write layer: they access the read only image and create an outside layer on top of it; if we add or move something in the image, the content is copies from the image to the container layer and modified in it - that is called *copy on write*.

```bash
# List all the changes done to the image through its history
# Each change is a layer, and we can see the command related to it 
docker image history nginx:latest
# Check many infos of the image
# Ports opened, command called when run, architecture, OS, author, etc.
docker image inspect nginx
```

### Image Tags + Pushing

Image tags are pointers to specific image commits, similarly as in Github.
Different tags might be pointing to the same version, eg.: `latest`, `2`, `2.1`, `2.1.3`, `mainline`.
However, the same version will have the same ID; if we download 2 tags pointing to the same image version, only one image will be downloaded even though we see two images (two tags).
Note that `latest`is usually the latest version in the official repos, but not necessarily in other personal repos.

```bash
# List all images on our machine
# Note that official ones have just the <image-name>
# The others usually are of the form <org|user-name>/<image-name>
docker image ls
# How to use image tagging
docker image tag --help
# docker image tag SOURCE_IMAGE[:TAG] TARGET_IMAGE[:TAG]
# Following that, we can tag any image we have, for instance
docker image tag nginx msagardia/nginx
# We can push our tagged image to Docker Hub if we log in
docker login
# username, pw
# if we want to log out: docker logout
# Push image
docker image push msagardia/nginx
# Now, the image is in Docker Hub
# By default, pushed images are public
# But we can create private image stoo via the web interface
# Create Repository > ... check Private; then, upload it
```

### Building Images: Dockerfile Basics

Images are created with a `Dockerfile`, which is scripts or recipy for creating images. The standard filename is `Dockerfile`, and we build the image with `docker build` in the directory we have the file. If the filename is another, we need to execute `docker build -f some-dockerfile`.

Each command (aka step) inside the `Dockerfile` consitutes a layer on top of the previous; typical commands appear in this order:

- `FROM`: we specify the basic linux distro we start building our image on
- `ENV`: define key:pair variables for later use
- `RUN`: download packages with package manager (eg., `apt-get`)
- `EXPOSE`: make desired port accessible; these still need to be opened with `-p`
- `CMD`: last command to run when launching container

For more information:
[https://docs.docker.com/engine/reference/builder/](Dockerfile reference).

Example of a `Dockerfile` from the cloned repository:
`~/git_repositories/udemy-docker-mastery/dockerfile-sample-1/Dockerfile`:
```docker
# Note that every command is a layer stacked on top of the previous
# First, a minimal linux distribution is loaded, like debian/alpine
FROM debian:stretch-slim

# Set environment variables: main way we set key:value pairs
ENV NGINX_VERSION 1.13.6-1~stretch
ENV NJS_VERSION 1.13.6.0.1.14-1~stretch

# Run commands in the linux distro: run shell scripts, shell commands, etc.
# Lines broken with \ and commands concatenated with &&; all after RUN is a layer
# Usually, first we install requires packages with the package manager
RUN apt-get update \
	&& apt-get install...
	...

# Typical lines: we point our log files to our stdout and stderr
# Docker handles our logging, but we need to spit out the logs to the shell
# Here, we output the nginx logs
RUN ln -sf /dev/stdout /var/log/nginx/access.log \
		&& ln -sf /dev/stderr /var/log/nginx/error.log

# We expose our desired ports here: since it's a webhost, ports 80 and 443
# But we still need to use -p or -P to open/forward them!
EXPOSE 80 443

# Final (obligatory) command we run every time we launch a container of the image
# Only one allowed, so the last one
CMD ["nginx", "-g", "daemon off;"]
```

### Building Images: Docker Builds

We generate the images by building/executing the `Dockerfile`. In the previous example, we would pull/download the `debian:stretch-slim` distro (`FROM`) and add the changes specified in the file to build the image.

```bash
cd ~/git_repositories/udemy-docker-mastery/dockerfile-sample-1
# Build image using Dockerfile in .
# Tag of image (-t) customnginx
docker image build -t customnginx .
# Note that each command/step produces an output
# When a step is finished, a hash is displayed
# Docker tracks hashes of the images to avoid repeating installations/builds of layers
# The hashed layers are saved in the cache, and used if no changes applied -> build is fast!
# Therefore, the order of the docker commands is important:
# always put at the end any any commands associated to possible changes

# Check that our image was built
docker image ls
```

Docker images are stored in
- Linux: `/var/lib/docker`
- Windows: `C:\ProgramData\DockerDesktop`
- Mac: `~/Library/Containers/com.docker.docker/Data/vms/0/`

### Building Images: Docker Builds

Usually we pull images from Docker Hub and extend them, for example the official `nginx`.

Example of a `Dockerfile` from the cloned repository:
`~/git_repositories/udemy-docker-mastery/dockerfile-sample-2/Dockerfile`:

```docker
# This shows how we can extend/change an existing official image from Docker Hub
FROM nginx:latest

# Change working directory to root of nginx webhost
# Using WORKDIR is preferred to using 'RUN cd /some/path'
WORKDIR /usr/share/nginx/html

# Copy the file index.html for our local host to our image
# nginx is a web server
COPY index.html index.html

# We don't have to specify EXPOSE or CMD because they're in my FROM
```

We run the default docker container of `nginx` first to check the web server and then build our custom `nginx` image.

```bash
# Run web server
docker container run -p 80:80 --rm nginx
# Open browser and go to localhost: Welcome page of nginx appears

# Build custom image/container with the previous Dockerfile
# Recall we copy the index.html file from the local host in dockerfile-sample-2
cd ~/git_repositories/udemy-docker-mastery/dockerfile-sample-2
docker image build -t nginx-with-html .
# Run container of new image
docker container run -p 80:80 --rm nginx-with-html
# Open browser again at localhost
# We should see a new message
# Now we can push the new image to Docker Hub
```

### Assignment

Assignment/Example: Build a Node.js image and push it to Docker Hub:
`~/git_repositories/udemy-docker-mastery/dockerfile-assignment-1/Dockerfile`

```docker
# These commands are written following the instructions in the Dockerfile
FROM node:6-alpine
EXPOSE 3000
RUN apk add --update tini
RUN mkdir -p /usr/src/app
WORKDIR /usr/src/app
COPY package.json package.json
RUN npm install && npm cache clean
COPY . . 
# This is the command '/sbin/tini -- node ./bin/www'
# Look at the docker cmd reference for more information
CMD ["tini", "--", "node", "./bin/tini"]
```

After writing the `Dockerfile`, we build it and run the container:

```bash
cd ~/git_repositories/udemy-docker-mastery/dockerfile-assignment-1/
# Edit the Dockerfile as above
# Build and run
docker build -t testnode .
docker container run --rm -p 80:3000 testnode
# Open browser: localhost
# Tag image
docker tag testnode msagardia/testing-node
docker push msagardia/testing-node
```

**Note: I could not make it work because there was a problem with Node.js**

### Prune

```bash
# System usage for conatiners/images/volumes
docker system df
# Remove everything that is not being used
docker system prune
# Remove images that are not being used
docker image prune -a
# Remove an image
docker image rm msagardia/testing-node
```

## Section 5: Section 5: Container Lifetime & Persistent Data (Volumes)

Some interesting resources mentioned in the course:
- [An introduction to immutable infrastructure](https://www.oreilly.com/radar/an-introduction-to-immutable-infrastructure/)
- [The 12 Factor App](https://12factor.net)
- [12 Fractured Apps](https://medium.com/@kelseyhightower/12-fractured-apps-1080c73d481c#.cjvkgw4b3)
- [Manage data in Docker](https://docs.docker.com/storage/)

Containers are usually **immutable** and **ephemeral**:
we can re-deploy containers from an image without any change.
However, the data produced inside the container are destroyed if we delete (`rm`) the container, they are not **persistent**, they are not stored in the image!
If we want to have **persistent** data that is stored, we have two options: **volumes** and **bind mounts**:

- **Volumes** are special locations outside from the container `union file system` (UFS), that can be accessible by several containers, but which are seen as a local path by the containers themselves.
- **Bind Mounts**: we share or mount a host path/file to a container, so that the container thinks it is its own file/path - bit it's host's.

### Persistent Data: Volumes

Volumes are created in the `Dockerfile` with the command `VOLUME`; for instance, for `mysql`: `VOLUME /var/lib/mysql`. The volumes outlive the containers when they are `rm`-d.

It is also possible to create volumes ahead of time with `docker volume create`. Checking `docker volume create --help`, we see that is necessary if we want different drivers (or driver options) and volume metadata.

```bash
# Remove all local volumes not used by at least one container
docker volume prune
# Pull mysql
docker pull mysql
# Inspect
# Look for Volumes
docker image inspect mysql
# Run container
docker container run -d --name mysql -e MYSQL_ALLOW_EMPTY_PASSWORD=True mysql
# Check that our container is running
docker container ls
# Inspect our running mysql container
# We see Volumes, and more importantly,
# Mounts: the source file location on the host and the destination on the container appear
# On Linux, we can navigate to that host file
# On Mac/Windows, docker has a linux virtual machine running, and the volumes ar ein there...
# When we do volume prune, the host volumes are removed
docker image inspect mysql
# List all volumes
docker volume ls
# Inspect a particular volume:
# Usually, 3 first letters from ls are enough, the TAB!
docker volume inspect 0a9
# When we start another mysql (of container which creates volumes)
# new volumes are created
# When containers are rm-d, volumes are still there!
# We know the volumes of each container,
# but, unfortunately, not the containers of each volume
# To add some clarification, we can use named containers
# Adding `-v /var/lib/mysql` is equivalent to `VOLUME /var/lib/mysql` in the Dockerfile
# If we pre-write `<vol-name>:` to the vol destination, then the volume has a name
docker container run -d --name mysql -e MYSQL_ALLOW_EMPTY_PASSWORD=True -v mysql-db:/var/lib/mysql mysql
# List volumes
# Volumes with names ar emuch clearer
docker volume ls
# Now, calling inspect is user-friendlier
docker volume inspect mysql-db
# VERY IMPORTANT:
# More importantly, everytime we rm a mysql container but create a new one
# we use the same volume/database if we specify it with -v!
# That is necessary to keep using the same database!
docker container stop mysql
docker container rm mysql
docker container run -d --name mysql -e MYSQL_ALLOW_EMPTY_PASSWORD=True -v mysql-db:/var/lib/mysql mysql
docker volume ls
```

### Persistent Data: Bind Mounting

Bind mounting maps a host file/directory to a container file/directory; the data is stored on the host, and the container has a link to it.
We cannot create them in the `Dockerfile`, we need to create them when calling `container run` with the option `-v`: this time, we pass the absolute host path instead of the name of the volume.
We can specify things also to be read only.

**That is very powerful, because we can basically develop our code on the host, in a preferred folder, and the container maps a local directory to the host's.**

```bash
# Mac/Linux
... run -v /Users/mikel/stuff:/path/container
# Windows
... run -v //c/Users/mikel/stuff:/path/container
```

Example in `~/git_repositories/udemy-docker-mastery/dockerfile-sample-2`:
```bash
# Got to sample
cd ~/git_repositories/udemy-docker-mastery/dockerfile-sample-2
# Check Dockerfile in the directory
# We do `COPY index.html index.html`
cat Dockerfile
# Run container with bind mount
# $(pwd) is the current directory
docker container run -d --name nginx -p 80:80 -v $(pwd):/usr/share/nginx/html nginx
# Open the browser and go to localhost
# The index.html file is opened!
# BUT: if we do not mount the local directory, teh default index.html is used
docker container run -d --name nginx2 -p 8080:80 nginx
# If we create any file on the host's linked directory $(pwd)
# and check the container directory /usr/share/nginx/html
# we see that the freshly created file is there!
touch test.txt
docker container exec -it nginx bash
cd /usr/share/nginx/html
ls -la
echo "this is a test file, shared by the host and the container" > test.txt
exit
less test.txt
# Thus, we can develop code on our host machine, bind mount the directory to the container, and that's it!
```

Example in `~/git_repositories/udemy-docker-mastery/bindmount-sample-1`:
Bind mount with a [Jekyll](https://jekyllrb.com) page defined locally.
Used image: [https://hub.docker.com/r/bretfisher/jekyll-serve](https://hub.docker.com/r/bretfisher/jekyll-serve)
```bash
# Create jekyll server using Bret Frisher's image
# We can stop it with Ctrl+C
# The local folder which contains al necesary files is mounted!
# That means, if we change a post content file on the host
# the visualized website will update!
docker run -p 80:4000 -v $(pwd):/site bretfisher/jekyll-serve
# Open browser and go to localhost
# Website appears
# Modfy the file in _posts/ and the post website will change
```

## Section 6: Docker-Compose for Starting Several Containers

With Docker-compose we can configure, start and handle several containers and their relationship!
First we need to generate a `docker-compose.yaml` configuration file using the YAML format.
Then, we use the `docker-compose` CLI tool.

Docker-Compose makes easy to run applications that need several services running in synch, e.g., web pages, etc.
Thus, the burden to run them is minimized and people can start trying them without needing to assemble complex environments or to develop in each of the required modules themselves!

However, note that docker-compose is not considered a production tool, but a development tool.
Usually, `docker stack` is used for production deployment.
Docker stack launches swarms, which are containerized services distrubuted in different nodes (see section below).
Swarms have many more functionalities under the hood than `docker-compose`, and they use the same docker compose files.

YAML: Yet another markup language.
Easy format to describe config files consisting of key-value pairs.
Some properties:
- indentation is important
- we have `key: value` pairs
- if a `key` has several values, we have a list where each element is preceeded by `-`
- if a `key` has several `key: value` pairs and values, these are note preceeded by `-`

More resources on YAML:
- [Get started](https://yaml.org/start.html)
- [Reference card](https://yaml.org/refcard.html)

Very good docker-compose file reference: [Compose file v3](https://docs.docker.com/compose/compose-file/compose-file-v3/).

Typical `docker-compose.yaml` structure:
```yaml
version: '3.1'  # if no version is specified then v1 is assumed.

services:  # containers, same as docker run
  servicename1: # a friendly name, this is also DNS name inside network
    image: # optional if you use build:
    command: # optional, replace the default CMD specified by the image
    environment: # optional, same as -e in docker run
    volumes: # optional, same as -v in docker run
	depends_on: # which other services need to be started too
  servicename2:
	...

volumes: # Optional, same as docker volume create

networks: # Optional, same as docker network create
```

Example of `docker-compose.yaml`:
```yaml
version: '3.1'
# same as 
# docker run -p 80:4000 -v $(pwd):/site bretfisher/jekyll-serve
services:
  jekyll:
    image: bretfisher/jekyll-serve
    volumes:
      - .:/site
    ports:
      - '80:4000'
```

### Docker-Compose CLI Tool

The `docker-compose` CLI tools comes automatically for Win/Mac, but it needs to be downloaded for Linux. It is thought for local development, bot for production. It uses `docker` on the background, so many CLI arguments are similar to those  in `docker`.

Most common commands:
```bash
# Setup all volumes and networks and start containers
# Interesting note: docker-compose adds to the containers, networks and volumes the directory name to prevent conflicts
docker-compose up
# Stop all containers and remove networks
docker-compose down
```

Example in `~/git_repositories/udemy-docker-mastery/compose-sample-2`:
```yaml
version: '3'

services:
  proxy:
    image: nginx:1.13 # this will use the latest version of 1.13.x
    ports:
      - '80:80' # expose 80 on host and sent to 80 in container
    volumes:
      - ./nginx.conf:/etc/nginx/conf.d/default.conf:ro
  web:
    image: httpd  # this will use httpd:latest
```

That `docker-compose.yaml` can be handled as follows:
```bash
cd ~/git_repositories/udemy-docker-mastery/compose-sample-2
ls # docker-compose.yml nginx.conf
docker-compose up
# Note that it is recommended to launch without -d, so that we get the logs in the shell
# since we are dealing with more tha one container usually
# Docker-compose nicely puts the container/service name before the log
browser(localhost) # It works!
Ctrl+C
docker-compose down
# If we want to start detached and get back the shell
docker-compose up -d
docker-compose logs
docker-compose down
# If we want to remove volumes
docker-compose down -v
# Some useful commands
docker-compose --helps
docker-compose ps
```

### Assignment 1: Set up a Drupal CMS

We set up a Drupal Content Management System (CMS similar to Wordpress).
We need two services: `drupal` and `postgres`.
Reading documentation is necessary, for instance
- to check the ports we need to expose in Drupal,
- clear the need for a pw for Postgres,
- and use named volumes.
We can do that by reading the documentation on Docker Hub or by pulling the base images and inspecting them:

```bash
docker pull drupal
docker image inspect drupal
docker pull postgres
docker image inspect postgres
```

The solution is in `~/git_repositories/udemy-docker-mastery/compose-assignment-1`:
```yaml
version: '2'

services:
  drupal:
    image: drupal:8.8.2
    ports:
      - "8080:80"
    volumes:
      - drupal-modules:/var/www/html/modules
      - drupal-profiles:/var/www/html/profiles       
      - drupal-sites:/var/www/html/sites      
      - drupal-themes:/var/www/html/themes
  postgres:
    image: postgres:12.1
    environment:
      - POSTGRES_PASSWORD=mypasswd

volumes:
  drupal-modules:
  drupal-profiles:
  drupal-sites:
  drupal-themes:
```

### Adding Image Build to Docker-Compose

We can build images at runtime, ie., when we do `docker-compose up`.
That is handy when

- we have complex `build` commands that would be better in a file
- we want to have very customized images built using specific config files present in our local folder every time

To build an image with `docker-compose`, we basically nest a `build` paragraph under the service we'd like to build. `build` accepts all expected values: `context`, `dockerfile`, etc. If the `image` of the service is not in the cache, docker builds it following `build`.

Exammple: we launch a website locally (localhost) using a template. This example is in `~/git_repositories/udemy-docker-mastery/compose-sample-3`:

```yaml
version: '2'
# based off compose-sample-2, only we build nginx.conf into image
# uses sample HTML static site from https://startbootstrap.com/themes/agency/
services:
  proxy:
    build:
      context: .
      dockerfile: nginx.Dockerfile
	# if no nginx-custom found, one is built according to build
	image: nginx-custom
    ports:
      - '80:80'
  web:
    image: httpd
    volumes:
      - ./html:/usr/local/apache2/htdocs/
```

How to manage that example with `docker-compose`:
```bash
cd ~/git_repositories/udemy-docker-mastery/compose-sample-3
# if we don't have the nginx-custom image, it is built
docker-compose up
browser(localhost) # refresh, web page template appears
# if the nginx-custom image is built, we can rebuild it in two ways
docker-compose build
# or
docker-compose up --build
# if we want to remove all created images
docker-compose down --rmi local
```

### Assignment 2: Image Building with Docker-Compose

A custom Drupal image is created out of the official image.

Example in `~/git_repositories/udemy-docker-mastery/compose-assignment-2`.

`Dockerfile`:
```docker
FROM drupal:8.6
# install git with apt-get package manager
# &&: if previous command succeds, include next one
# -y: yes to all
# apt-get creates some cache which increases unnecessarily our image
# thus, remove cache content: always in /var/lib/apt/lists/
RUN apt-get update && apt-get install -y git \
    && rm -fr /var/lib/apt/lists/*

WORKDIR /var/www/html/themes

# Git clone only one branch with a only the latest depth/commit: we save space and time
# Docker runs as admin (sudo), and we sometimes need to change some user ownership
# Here, we change group and user of all files and dirs recursively (-R) in bootstrap to be
# www-data:www-data (group:user) 
RUN git clone --branch 8.x-3.x --single-branch --depth 1 https://git.drupalcode.org/project/bootstrap.git \
    && chown -R www-data:www-data bootstrap

WORKDIR /var/www/html/
```

`docker-compose.yaml`:
```yaml
version: '2'

services:
  drupal:
    image: drupal-custom
    # build . -> Dickefile is here, build it w/o any special options
    build: . 
    ports:
      - "8080:80"
    volumes:
      - drupal-modules:/var/www/html/modules
      - drupal-profiles:/var/www/html/profiles       
      - drupal-sites:/var/www/html/sites      
      - drupal-themes:/var/www/html/themes
  postgres:
    image: postgres:12.1
    environment:
      - POSTGRES_PASSWORD=mypasswd
    volumes:
      - drupal-data:/var/lib/postgresql/data

volumes:
  drupal-data:
  drupal-modules:
  drupal-profiles:
  drupal-sites:
  drupal-themes:
```

`docker-compose` commands:
```bash
docker-compose up
browser(localhost:8080)
# Drupal installation interface appears
# We install Drupal and select the theme bootstrap
# Create a post/welcome page, etc.
# We can setup a website locally now, similarly as we do it with Wordpress
Ctrl+C
docker-compose down
# If we start it again, vrything is still there
# beacuse we preserve the volumes
docker-compose up
browser(localhost:8080)
```

## Section 7: Swarm Mode: Built-In Orchestration

Swarm mode is a server clusting solution, aka orchestration.
It makes possible to scale up/down our applications and provides with tools to manage all the data and processes generated and run during the whole lifecycle.
Swarm is built on top of docker but does not affect it, so that we can use any other orchestration system on top of it, like Kubernetes.

How everything works:
- We have several nodes/servers: physical computers or virtual machines
- The whole setup of nodes is the swarm
- Nodes can be managers and/or workers
- Managers have the permissions to change and handle the swarm
- Managers send work to the workers and they can be also workers
- Managers usually share a database of states, which is the communication protocol RAFT; they can make synchronized decissions very fast
- Managers have cryptographic identity with a certificate
	- This way, we can dynamically join managers if we have the correct certificate
- One of the managers is the leader
- Workers communicate with each other with the gossip protocol
- We can dynamically change the nodes to be worker/manager
- Workers are constantly reporting to managers and asking for work if idle
- Whereas with `docker run` we can execute locally one container, with a swarm manager we can do `docker service`, which is able to execute several tasks = containers in different nodes, ie., several replicas of the prototype container are run in different nodes; this way, we scale up/down. The manager spreads the tasks through the available nodes.
- The manager node has several components
	- API: accepts and stores commands
	- Orchestrator: it reconciles the current and desired states (due to the accepted commands) by creating tasks
	- Allocator: allocation of IP addresses to tasks
	- Scheduler: nodes assigned to tasks
	- Dispatcher: distribute tasks to workers

### Creating a Service with a Single Node

A service is an image being executed in several containers, which are doing the equivalent job.
Often the analogy *cattle-pet* is used to differentiate between *service-container*.
The number of containers is called replicas.

We have three important `docker` commands to interact with a swarm:
- `swarm`: it refers to the entire system of nodes, the whole swarm.
- `node`: it refers to a specific node in the whole system.
- `service`: it referes to a class of containers that are run on nodes; a service is decomposed in tasks, which are the containers executing the image. Importantly, service properties (replicas, etc.) can be `updated` without stopping them! This way, *blue-green* deployments are possible.

```bash
# Check if sawrm mode is active
# Look in output for "Swarm: active/inactive"
docker info
# Initialize swarm
# We see instructions to
# - add managers
# - add workers
docker swarm init
# List available nodes (one at the moment)
docker node ls
# Check all available options/commands with docker node
# - demote: demote node from manager
# - inspect
# - ls
# - promote: promote node to manager
# - ps
# - rm: remove nodes from swarm
# - update
docker node help
# Check all available options/commands with docker swarm
# - init: initialize swarm
# - join: join a swarm as a node
# - join-token
# - leave: leave the swarm
# - unlock
# - unlock-key
# - update
docker swarm --help
# Check all available options/commands with docker service
# - create
# - inspect
# - logs
# - ls: list services
# - ps: list the TASKS of the service (=containers running)
# - rm
# - rollback
# - scale
# - update: change properties of the service running
docker service --help
# Create a service: ping to the Google server 8.8.8.8
# The ID is returned, but we can always get it with docker service ls
docker service create alpine ping 8.8.8.8 --detach true
# List services running (and their names & ids)
docker service ls
# List the tasks of a service
# Tasks are containers with the name: service_name.X.id, X = 1, 2, ...
# We can then list them with docker container ls
docker service ps
docker service ps <service id/name>
# Change/Update properties of service
# In this case, increase the replicas to 3 (tasks or container of service)
docker service update <service id> --replicas 3 --detach true
# IMPORTANT: update has many options
# which make possible to modify the service without stopping it!
docker service update --help
# We can manually stop tasks/containers of a swarm
# but they are restarted again!
# Therefore, we need to remove the service to stop the containers
docker container ls
docker container rm -f <name>
docker service ps <name>
# Remove a service and stop its tasks/containers
docker service rm <id/name>
```

### Creating a Service with Three Nodes

We need actually three nodes for that: either VMs or real/physical computers.
In order to interact with different nodes, the CLI tool `docker-machine` is used.
This tool comes installed for Windows/Mac, but needs t obe installed for Linux: [Install Docker Machine](https://docs.docker.com/machine/install-machine/).

Four options are explained in the course:

1. `labs.play-with-docker.com`: log in with Docker Hub credentials; it is for testing only, since we need to reastrat every 4h. We create node instances by clicking on `new instace`.

2. `docker-machine` + VirtualBox: VM that runs locally; we just need to install VirtualBox and do nothing more but run `docker-machine create myNode`! Then, lightweight Linux VMs are started (BusyBox) in the background. Note tha VMs consume memory and CPU. Note that the first time on a Mac we need to give permissions to Oracle in the System Pref. > Security & Privacy, and we need to restart too.

3. `DigitalOcean.com` servers. Note that docker needs to be installed on our instances; we can check how to do it on [https://get.docker.com/](https://get.docker.com/): `curl -fsSL https://get.docker.com -o get-docker.sh`

4. Roll your own setup. Note that docker needs to be installed on our instances.

Usually, the VirtualBox option is taken and we run several instances of VMs. In addition, docker comes already installed.

Creating and interacting with nodes using VirtualBox:
```bash
# After VirtualBox has been installed + permissions given + computer restarted
# Create a new node called node1 (by default a BusyBox instance)
# It might take some seconds and we get the shell back
docker-machine create node1
# List nodes
docker-machine ls
# Two options to access the machines/nodes
# 1. ssh: we connect to it through ssh
# 2. env: we get some output; if we execute it (or just the last line),
# our docker CLI talks to the node that corresponds
docker-machine ssh node1 # exit for exiting
docker-machine env node1 # watch out: shell env variables modified
# Remove a node
docker-machine rm node1
# We create 3 nodes and connect to them via ssh from 3 different shells
docker-machine create node2
docker-machine create node3
docker-machine ssh node1
docker-machine ssh node2
docker-machine ssh node3
# In all nodes, we can get version info
docker version
# In a node, say node1, we initialize the swarm
docker swarm init
# We will probably get an error
# because the IP to advertise to the rest of the nodes cannot be chose
# We can do it with --advertise-addr <IP>, using one of the suggested IPs
# @host, we do
docker-machine ip node1
docker-machine ip node2
docker-machine ip node3
# We check that all IPs are different
# We select a node, say node1, and copy its IP
# @node1, we do
docker swarm init --advertise-addr <IP>
# That node1 will become the manager, as the output says 
# After that we get a docker swarm join output
# that we copy and paste in the other nodes: node2, node3
docker swarm join --token <token> <IP>:<port>
# Worker nodes cannot access/control the swarm from the CLI
# @node2 (worker): this will return an error
docker node ls
# @node1 (manager): this will return the list of all nodes1-2-3
# node we're talking to denotes with *
# manager nodes are makerd as Reachable / Leader
docker node ls
# We can promote worker nodes to be manager from a manager node, eg
# @node1
docker node update --role manager node2
# Another option is to get a token from a manager node, eg:
# @node1
docker swarm join-token manager
# Copy join token command and paste it on a worker node, eg:
# @node3
docker swarm leave
docker swarm join --token <token> <IP>:<port>
# Now, we can create a service with 3 replicas!
docker service create --replicas 3 alpine ping 8.8.8.8
# We can see we have 3 tasks/containers running
docker service ls
docker service ps <service name>
# Note that most of the management will be done in node1 (manager)
# To remove, get list of nodes and rm them
docker-machine ls
docker-machine rm node1
docker-machine rm node2
docker-machine rm node3
# Now, we should not have anything
docker-machine ls
docker service ls
docker node ls
```

## Section 8: Swarm Basic Features


### Overlay Networking

We can create a swarm-wide bridge network with `--driver overlay`, which is essentially like a VLAN.
Nodes within the swarm can communicate with each other seamlessly.
The services get a DNS and a virtual IP (VIP) with which the outsiders communicate.

CMS (drupal) for website with assiciated database (postgres):
```bash
# Initialize swarm (in this example on the host, only a node: host)
docker swarm init
# Create overlay network with name mydrupal
docker network create --driver overlay mydrupal
# List networks
docker network ls
# Create a service with the postgress image
docker service create --name psql --network mydrupal -e POSTGRES_PASSWORD=mypass postgres
# Check the tasks
docker service ps TAB # psql
# Check the logs of the task
docker container logs TAB # psql.1.X
# Create another service with drupal
docker service create --name drupal --network mydrupal -p 80:80 drupal
# List all services: drupal, psql
docker service ls
```

### Routing Mesh

If in the previous example we launch three nodes and open the CMS site, all three nodes display the site, although only one is running it. In oder words, if we execute a service in 3 nodes on port X, we can access it in all 3 nodes on port X.

That is because we have a **routing mesh**: all incoming communications are interfaced by all nodes in the swarm; that makes everything very comfortable:
the load balancer can change the work distribution in the swarm, but the user outside is able to access it as before.

The **routing mesh** in docker is stateless: no cockies are saved for different sessions, so we start fresh.

### Assignment: Docker's Distributed Voting App

Very nice and typical example shown quite often.
Voting app composed by several services, each of them written in a specific language.
Voting is cats vs. dogs.

Explanation of how to do is given in the following folder of the course

`~/git_repositories/udemy-docker-mastery/swarm-app-1/`

Notes:

- Write down the commands first in the `README.md` and then execute them.
- An image of the architecture is in the folder.
- Images are in Docker Hub; usually, we build images locally, then launch the swarm.
- A `backend` and `frontend` overlay network are needed.
- The database server should use a named volume for preserving data. Use the new `--mount` format to do this: `--mount type=volume,source=db-data,target=/var/lib/postgresql/data`
- Services with their properties listed in the `README.md`

```bash
# We need 3 nodes: create them
# @host
docker-machine create node1
docker-machine create node2
docker-machine create node3
# @host: get IP
docker-machine ip node1
# Connect via SSH to the nodes
docker-machine ssh node1
docker-machine ssh node2
docker-machine ssh node3
# @node1: manager
docker swarm init --advertise-addr <IP>
# @node2, node3: paste the worker tokens
docker swarm join --token <token> <IP>:<port>
# @node1: manager
docker node ls
# @node1: Create the networks: backendm frontend
docker network create -d overlay backend
docker network create -d overlay frontend
# @node1: Service 1
docker service create --name vote -p 80:80 --network frontend --replicas 2 bretfisher/examplevotingapp_vote
# @node1: Service 2
docker service create --name redis --network frontend redis:3.2
# @node1: Service 3
docker service create --name db --network backend -e POSTGRES_HOST_AUTH_METHOD=trust --mount type=volume,source=db-data,target=/var/lib/postgresql/data postgres:9.4
# @node1: Service 4
docker service create --name worker --network frontend --network backend bretfisher/examplevotingapp_worker:java
# @node1: Service 5
docker service create --name result --network backend -p 5001:80 bretfisher/examplevotingapp_result
# @node1: Check that we have all the services with their replicas
docker service ls # db, redis, result, vote, worker
# @node1: We can check deeper 
docker service ps redis
docker service logs worker
# Application
# @host
docker-machine ip node1
# Open browser: insert IP:5000
# We can vote
# Open browser: insert IP:5001
# Results appear
# Stopping
# @node1
docker service rm db redis result vote worker
docker service ls
# @node2, node3
docker swarm leave
# @node1
docker swarm leave --force
# @host
docker-machine rm node1
docker-machine rm node2
docker-machine rm node3
```

### Stacks of Services: Compose for Swarms

Similarly as `docker-compose` could be used to launch several containers on a host, `docker stack` accept compose files as definition and launch files of swarms consisting of services, networks, and volumes.

Instead of using `docker service create` several times, we use `docker stack deploy`, which calls a `*.yaml` file that defines the swarm.
The syntax is almost identical to the one used by `docker-compose`. Exceptions:
- we need to use at least version 3
- we have new keywords
  - `deploy`: for deploying services
    - `update_config`: what happens when I do an update: `parallelism`, `delay`
    - `restart_policy`
    - `placement`
- building does not happen in production swarm, but before, so:
  - `docker-compose` ignores `deploy` keywords form the YAML
  - `docker stack` ignores `build` keywords from the YAML

We re-do the same example as before, but use a YAML file instead.
The YAML file is in
`~/git_repositories/udemy-docker-mastery/swarm-stack-1/`
```bash
# We need 3 nodes: create them
# @host
docker-machine create node1
docker-machine create node2
docker-machine create node3
# @host: get IP
docker-machine ip node1
# Connect via SSH to the nodes
docker-machine ssh node1
docker-machine ssh node2
docker-machine ssh node3
# @node1: manager
docker swarm init --advertise-addr <IP>
# @node2, node3: paste the worker tokens
docker swarm join --token <token> <IP>:<port>
# @node1: manager
docker node ls
# @node1: manager
# we need to copy somehow the stack file
# or just open a vi and copy the YAML content to it
mkdir srv
cd srv
mkdir swarm-stack-1
cd swarm-stack-1
touch example-voting-app-stack.yml
vi example-voting-app-stack.yml
docker stack deploy -c example-voting-app-stack.yml voteapp
# list stacks
docker stack ls
# list tasks running where
docker stack ps <stack name>
docker stack ps voteapp
# service ls for the stack we want
docker stack services voteapp
# list networks
docker network ls
# Open browser: insert IP:5000
# We can vote
# Open browser: insert IP:5001
# Results appear
# We have a new service, called visualizer!
# Open browser: insert IP:8080
# Graphical display of the swarm and its nodes done!
# We can modify the YAML file and deploy it again (with the same name): 
# The services changed will be updated
# Example (@node1)
vi example-voting-app-stack.yml
# vote: replicas: 5
docker stack deploy -c example-voting-app-stack.yml voteapp
# We see how services are updated :)
# Stopping
# @node1
docker stack rm <stack name>
docker stack rm voteapp
docker service ls
# @node2, node3
docker swarm leave
# @node1
docker swarm leave --force
# @host
docker-machine rm node1
docker-machine rm node2
docker-machine rm node3
```

`example-voting-app-stack.yaml`:
```yaml
version: "3"
services:

  redis:
    image: redis:alpine
    networks:
      - frontend
    deploy:
      replicas: 1
      update_config:
        parallelism: 2
        delay: 10s
      restart_policy:
        condition: on-failure
  db:
    image: postgres:9.4
    volumes:
      - db-data:/var/lib/postgresql/data
    networks:
      - backend
    environment:
      - POSTGRES_HOST_AUTH_METHOD=trust
    deploy:
      placement:
        constraints: [node.role == manager]
  vote:
    image: bretfisher/examplevotingapp_vote
    ports:
      - 5000:80
    networks:
      - frontend
    depends_on:
      - redis
    deploy:
      replicas: 2
      update_config:
        parallelism: 2
      restart_policy:
        condition: on-failure
  result:
    image: bretfisher/examplevotingapp_result
    ports:
      - 5001:80
    networks:
      - backend
    depends_on:
      - db
    deploy:
      replicas: 1
      update_config:
        parallelism: 2
        delay: 10s
      restart_policy:
        condition: on-failure

  worker:
    image: bretfisher/examplevotingapp_worker:java
    networks:
      - frontend
      - backend
    depends_on:
      - db
      - redis
    deploy:
      mode: replicated
      replicas: 1
      labels: [APP=VOTING]
      restart_policy:
        condition: on-failure
        delay: 10s
        max_attempts: 3
        window: 120s
      placement:
        constraints: [node.role == manager]

  visualizer:
    image: dockersamples/visualizer
    ports:
      - 8080:8080
    stop_grace_period: 1m30s
    networks:
      - frontend
    volumes:
      - /var/run/docker.sock:/var/run/docker.sock
    deploy:
      placement:
        constraints: [node.role == manager]

networks:
  frontend:
  backend:

volumes:
  db-data:

```

### Secrets for Swarm Services

A secret is anything that is not supposed to be public, not only usernames, passwords and keys, but anything that is not supposed to be on our webpage!

Docker creates secrets and encrypts them automatically within a swarm.

Secrets are assigned to services, and only containers in assigned services can see them.

Secrets are in-memory only, although they look like files: `/run/secrets/<secret_name>`; we can have also aliases: the same in-memory secret with several memory names.

`docker-compose` does not have in-memory secrets, but it makes a work-around with file-based secrets, which are not really secure.

We can create a secret in a swarm in two ways, both with drawbacks: 
- passing a file; if the file is on disk, it is a potential risk
- passing a value at the command line; a malicious user could check for the pw in the bash history

```bash
# We create 2 nodes
# @host
docker-machine create node1
docker-machine create node2
# @host: get IP
docker-machine ip node1
# Connect via SSH to the nodes
docker-machine ssh node1
docker-machine ssh node2
# @node1: manager
docker swarm init --advertise-addr <IP>
# @node2, node3: paste the worker tokens
docker swarm join --token <token> <IP>:<port>
# @node1: manager
# We create a file with the content of the secret
docker node ls
mkdir srv
cd srv
mkdir secrets-sample-1
cd secrets-sample-1
touch psql_user.txt
vi psql_user.txt # mypsqluser
# Create secret from file
# secret id is returned
docker secret create psql_user psql_user.txt
# Now we create a secret for the pw
# but this time passing a value thorugh command line
# we echo a pw, '-' means 'read from input' 
# secret id is returned
echo "myDBpassWORD" | docker secret create psql_pass -
# List secrets
docker secret ls
# Inspect a secret: name, is, creation time, ... NEVER the secret value itsefl!
docker secret inspect <secret name>
# Secrets are stored in-memory, in the docker database
# we assign them to services, so that they become visible (decrypted) to them
# We start a postgres service and assign our secrets to it
docker service create --name psql --secret psql_user --secret psql_pass -e POSTGRES_PASSWORD_FILE=/run/secrets/psql_pass -e POSTGRES_USER_FILE=/run/secrets/psql_user postgres
# Check service and its containers
docker service ps psql # psql.1
# We open a bash shell in the container of the service
docker exec -it psq TAB bash
docker exec -it <container name> bash
# We can see the decrypted values of the secrets!
# These seem to be files, but they are really temp fs in-memory files
cd /run/secrets
ls # psql_user psql_pass
cat psql_user # mypsqluser
cat psql_pass # myDBpassWORD
exit
# @node1
# We check that the service is running correctly
# That's because the secrets are correctly set
docker service ps psql
# Now, we could remove the secrets with update
# however, for databases that does not make sense
# Additionally, when we do that, the service/container is re-deployed
# and here the re-deployment fails, because we have no secret value for the user
docker service update --secret-rm psql_user psql
```

### Secrets for Swarm Stacks (in Compose YAML Files)

We can define secrets for stacks launched with compose YAML files:
- We need to use the `docker-compose` file version `>=3.1` for using secrets with stacks.
- Version `>=3.0` is necessary for stacks alone.
- We have a new section in the YAML file: `secrets`.
- In the `secrets` section we can use `file`-based secrets or `external` secrets, as in the CLI.
- If `file`-based secrets used, remember to remove the original files!

Example in
`~/git_repositories/templates/docker/udemy-docker-mastery/secrets-sample-2`:
```bash
cd ~/git_repositories/templates/docker/udemy-docker-mastery/secrets-sample-2
ls # docker-compose.yml psql_password.txt psql_user.txt
# @host
# We create a swarm on the host as a node: we don't need the IP
docker swarm init
docker node ls # docker-desktop
# We deploy stack with secrets in YAML file
docker stack deploy -c docker-compose.yml mydb
docker secret ls # mydb_psql_password mydb_psql_user
# Remove stack: we see that secrets are removed too
docker stack rm mydb
```

`~/git_repositories/templates/docker/udemy-docker-mastery/secrets-sample-2/docker-compose.yaml`
```yaml
version: "3.1"

services:
  psql:
    image: postgres
    secrets:
      - psql_user
      - psql_password
    environment:
      POSTGRES_PASSWORD_FILE: /run/secrets/psql_password
      POSTGRES_USER_FILE: /run/secrets/psql_user

secrets:
  psql_user:
    file: ./psql_user.txt
  psql_password:
    file: ./psql_password.txt
```

## Section 9: Swarm App Lifecycle

`docker-compose` is not considered a production tool, but rather a development tool.
The equivalent production deployment tool would be `docker stack`, which is for swarms.
That is so because we have many functionalities in swarms under the hood compared to what we get with `docker-compose`: a database of states, overlay networks, secrets, etc.
However, `docker-compose` simulates some of the functionalities, although for testing (not securely), such as secrets:

If we launch a compose file with file-based secrets, `docker-compose` is going to bind-mount the secret files:

```bash
docker-compose up -d
docker-compose exec psql cat /run/secrets/psql_user
```

### Launching several `docker-compose` files

We can have several `docker-compose` YAML files that are combined depending on the environment we're in: **development**, **testing/CI**, **production**.

Example in
`~/git_repositories/templates/docker/udemy-docker-mastery/swarm-stack-3`:
Files we have
```bash
# Base docker-compose file: defaults across all envs
# Here we have image names used in the other files
docker-compose.yml
# If named like this, docker-compose will use it
# when we do docker-compose up after the previous file
# Usually this file is for the local development
docker-compose.override.yml
# Configuration for testing/CI
# Here we use docker-compose -f ... up: local testing
# Compared to the previous, here for instance we might not have named volumes, fake pws, ...
docker-compose.test.yml
# Configuration for production
# Here we use docker-compose -f ... config: it outputs the production YAML
docker-compose.prod.yml
# The rest of the files are used by docker
Dockerfile
psql-fake-password.txt
```

What we would then do:
```bash
# Default files are loaded and combined/overlaid:
# docker-compose.yml docker-compose.override.yml
docker-compose up -d
# Test env files are loaded and combined/overlaid:
# docker-compose.yml docker-compose.override.yml
docker-compose -f docker-compose.yml docker-compose.test.yml up -d
# Production: we do not use docker-compose up, but swarm/stack
# Here, we generate the YAML for swarm/stack
# Production env files are loaded and combined/overlaid:
# docker-compose.yml docker-compose.prod.yml
docker-compose -f docker-compose.yml docker-compose.prod.yml config
docker-compose -f docker-compose.yml docker-compose.prod.yml config > output.yml
```

### Service Updates

Notes:
- Service upates limit downtime, but real downtime is avoided through tests!
- Updates replace containers completely, take that into account.
- Updates have 70+ options that are combined usually with `-add` or `-rm`
- `stack deploy` of a running sttack is an update
- We can update several things on an `update` command

Typical update commands:
```bash
# Create the service
docker service create -p 8080:80 --name web nginx:1.13.7
docker service ls
# Change number of replicas
# We can update several services at once: web=5 api=3
docker service scale web=5
# Updating image: each replica is updated one at a time
docker service update --image nginx:1.13.6 web
# Change a port: that means remove old and add new
docker service update --publish-rm 8080 --publish-add 9090:80 web
# We can similarly update several things in a command line
docker service update --env-add NODE_ENV=production --prublish-rm 8080 <servicename>
# We can force re-balancing by commanding updates that do not change anything
docker service update --force web
# Stop service
docker service rm web
```

### Healthchecks

When going into production, it is very recommended using health checks:
- Health checks are check commands executed in containers and services with agiven frequency which expect an outcome value of 0
- These are commands passed as an option in `docker container run`, `docker service create`, in the `Dockerfile`, or in `docker-compose`
- The container health states are: `starting`, `healthy`, `unhealthy`
- A service replaces its unhealthy tasks
- We have several options:
  - `--health-cmd`: the command itself, eg "curl -f http://localhost || exit 1"
  - `--health-interval`: every how many seconds the healthcheck is executed
  - `--health-timeout`
  - `--health-retries`
  - ...
- The command in `Dockerfile` is `HEALTHCHECK`, the option token change a little bit
- The command/section in the YAML of `docker-compose` is `healthcheck`
- We can peek thier result with 
  - `docker container ls`: `STATUS` column
  - or with `docker container inspect`: result of last 5 checks shown in `Health` section

Examples
```bash
# Run container with healthcheck
docker container run --name p2 -d --health-cmd="pg_isready -U postgres || exit 1" postgres
# Look at healthcheck status
docker container ls
docker container inspect p1
# Run a service with a healthcheck
# Startup is slower, because health checks force 30 starting time
docker service create --name p2 --health-cmd="pg_isready -U postgres || exit 1" postgres
```
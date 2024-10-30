# Guide to Using Docker

Docker is a platform that allows developers to automate the deployment of applications inside lightweight, portable containers. These containers bundle an application with all its dependencies and environment, ensuring that the application runs consistently regardless of where it's deployed.

## What is Docker?

Docker simplifies application deployment by using containers, which are isolated environments that run an application along with all its required libraries and dependencies. Docker containers are highly portable and can run on any system that has Docker installed, making them ideal for consistent development, testing, and deployment.

### Difference Between a Docker Image and a Container

- **Docker Image**: An image is a read-only template that contains the instructions for creating a Docker container. It includes everything needed to run an application, including the code, runtime, libraries, environment variables, and configurations. Images are stored in a registry (like Docker Hub) and are the blueprints for containers.

- **Docker Container**: A container is a runnable instance of a Docker image. It is a lightweight, standalone, executable package that includes everything needed to run the application. Containers can be started, stopped, moved, and deleted, making them very flexible and easy to manage.

## Installing Docker Engine
If you do not have a linux device you will need a virtual machine (VM) such as WSL.

### Docker on Ubuntu

To install Docker on an Ubuntu system, follow these steps:

#### Add Docker's Official GPG Key:

```bash
sudo apt-get update

sudo apt-get install apt-transport-https ca-certificates curl software-properties-common

sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -

sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"


sudo apt-get install ca-certificates curl

sudo install -m 0755 -d /etc/apt/keyrings

sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc

sudo chmod a+r /etc/apt/keyrings/docker.asc

sudo apt-get update
sudo apt-get install docker-ce

sudo systemctl start docker
sudo systemctl enable docker

sudo docker --version

sudo usermod -aG docker $USER

newgrp docker

docker run hello-world

```

#### Add the repository to Apt sources:
To add the Docker repository to your system's APT sources list, use the following command:

```bash
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null


sudo apt-get update
```



## Installing NVIDIA Docker for GPU Support
Docker supports GPU acceleration through the NVIDIA Container Toolkit. Follow these steps to install it:

### Add the NVIDIA Docker Package Repositories:
```bash

distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
```

### Install the NVIDIA Docker package
```bash
sudo apt update
sudo apt install -y nvidia-docker2
```

### Restart Docker to apply the changes
```bash
sudo systemctl restart docker
```

## Useful Docker Commands
Here are some common Docker commands that are useful for working with containers and images:

### Building a Docker Image
To build a Docker image from a Dockerfile:
```bash
sudo docker build -t spear:v1 .
```
### Running a Docker Container
To run a Docker container from an image:

```bash
docker run -it -v <directory of github repo>:/home spear:v1
```

### Deleting a Docker Image
To delete a Docker image by its ID:

```bash
docker rmi -f <image id>
```

### Running a Container with Volume Mount
To run a container with a mounted volume, allowing data sharing between the host and the container:

```bash
docker run -it -v <directory of github repo>:/home spear:v1
```

### Viewing Docker Images on Your Computer
To list all Docker images available on your system:

```bash
docker image ls
```

### Viewing Running Docker Containers
To list all currently running Docker containers:

```bash
docker ps
```

